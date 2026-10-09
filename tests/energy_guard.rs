//! The C API against a real Energy Guard over loopback: SHIP, TLS, SPINE, LPC.
//!
//! The device is driven only through the `extern "C"` functions, exactly as C firmware
//! would drive it; the control box is the crate's own `EnergyGuardActor`.

use std::ffi::{CStr, CString, c_char, c_int, c_void};
use std::sync::Mutex;
use std::time::Duration;

use eebus::cert::{self, CertParams};
use eebus::model::{DeviceType, EntityType};
use eebus::runtime::{Hub, HubEvent, Node, TrustStore};
use eebus::spine::{Engine, LocalDevice, LocalEntity};
use eebus::tls::ShipTls;
use eebus::usecases::limitation::{self, EnergyGuardActor, GuardEvent, LimitWrite};
use eebus::usecases::lpc;
use eebus::usecases::monitoring::{self, MonitoringApplianceActor, MonitoringEvent};
use eebus::usecases::mpc;
use eebus_c::*;

static LIMITS: Mutex<Vec<(c_int, c_int, f64)>> = Mutex::new(Vec::new());
static PEERS: Mutex<Vec<(String, c_int)>> = Mutex::new(Vec::new());

unsafe extern "C" fn on_limit(_: *mut c_void, state: c_int, limited: c_int, watts: f64) {
    LIMITS.lock().unwrap().push((state, limited, watts));
}

unsafe extern "C" fn on_peer(_: *mut c_void, ski: *const c_char, connected: c_int) {
    let ski = unsafe { CStr::from_ptr(ski) }.to_string_lossy().into_owned();
    PEERS.lock().unwrap().push((ski, connected));
}

fn control_box() -> (Engine, eebus::model::FeatureAddress, eebus::model::FeatureAddress) {
    let mut device =
        LocalDevice::new("i:46925", "ControlBox-1", DeviceType::ElectricitySupplySystem).unwrap();
    device
        .add_entity(
            LocalEntity::new([1], EntityType::GridGuard)
                .with_feature(limitation::client_feature(1))
                .with_feature(limitation::device_diagnosis_feature(2)),
        )
        .unwrap();
    let client = device.address_of(&[1], 1);
    let diagnosis = device.address_of(&[1], 2);
    let mut engine = Engine::new(device);
    engine.add_use_case([1], 1, &lpc::ENERGY_GUARD);
    engine.add_use_case([1], 1, &mpc::MONITORING_APPLIANCE);
    (engine, client, diagnosis)
}

fn wait_for(what: &str, mut done: impl FnMut() -> bool) {
    for _ in 0..100 {
        if done() {
            return;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    panic!("timed out waiting for {what}");
}

#[tokio::test(flavor = "current_thread")]
async fn a_limit_reaches_the_firmware_and_survives_a_restart() {
    let dir = tempfile::tempdir().unwrap();
    let state_dir = CString::new(dir.path().to_str().unwrap()).unwrap();
    let product = CString::new("HeatPump-Test").unwrap();
    let port = 47_000 + (std::process::id() % 1000) as u16;

    // ---- the control box's identity, trusted by the device in advance
    let box_trust = TrustStore::new();
    let control = Node::new(
        "i:46925_u:ControlBox-1",
        ShipTls::new(cert::self_signed(CertParams::new("i:46925_u:ControlBox-1")).unwrap()),
        box_trust.clone(),
    );
    let box_ski = CString::new(control.ski().to_string()).unwrap();

    // ---- the device, through the C API only
    let mut config = std::mem::MaybeUninit::<EebusDeviceConfig>::uninit();
    let mut config = unsafe {
        eebus_device_config_init(config.as_mut_ptr());
        config.assume_init()
    };
    config.state_dir = state_dir.as_ptr();
    config.product = product.as_ptr();
    config.port = port;
    config.mdns = 0;
    config.failsafe_watts = 4_200.0;
    config.nominal_max_watts = 11_000.0;
    config.on_limit = Some(on_limit);
    config.on_peer = Some(on_peer);

    let mut device = std::ptr::null_mut();
    assert_eq!(unsafe { eebus_device_new(&config, &mut device) }, 0);
    let ski = unsafe { CStr::from_ptr(eebus_device_ski(device)) }
        .to_str()
        .unwrap()
        .to_string();
    assert_eq!(ski.len(), 40);
    assert_eq!(unsafe { eebus_device_trust(device, box_ski.as_ptr()) }, 0);
    assert_eq!(unsafe { eebus_device_start(device) }, 0, "{:?}", last_error());
    assert_eq!(unsafe { eebus_device_set_power(device, 1_500.0) }, 0);
    assert_eq!(unsafe { eebus_device_set_energy(device, 12_345.0, -1.0) }, 0);

    // Init: the failsafe value applies until the controller is heard from.
    wait_for("the initial report", || !LIMITS.lock().unwrap().is_empty());
    assert_eq!(LIMITS.lock().unwrap()[0], (STATE_INIT, 1, 4_200.0));

    // ---- the control box dials and writes 3 kW
    box_trust.trust(ski.parse().unwrap());
    let (engine, client, diagnosis) = control_box();
    let mut hub = Hub::new(control, engine);
    let mut guard = EnergyGuardActor::new(lpc::DIRECTION, client.clone(), diagnosis, hub.now());
    let mut monitor = MonitoringApplianceActor::new(client);
    let mut power = None;
    hub.dial(([127, 0, 0, 1], port).into());

    let mut stage = 0;
    for _ in 0..400 {
        let now = hub.now();
        hub.wake_at(guard.poll_timeout());
        hub.wake_at(now + Duration::from_millis(200));
        let reports = match hub.next().await.unwrap() {
            HubEvent::PeerDiscovered { device, .. } => {
                guard.require(&device, Some(LimitWrite::active(3_000.0)), now);
                let remote = hub.engine().peer(&device).expect("discovered");
                let unit = monitoring::locate(
                    remote,
                    mpc::MONITORED_UNIT.name,
                    mpc::MONITORED_UNIT.actor,
                )
                .expect("the device plays the MPC Monitored Unit");
                assert!(unit.is_usable(), "electrical connection and measurement found");
                monitor.attach(hub.engine_mut(), unit, now);
                Vec::new()
            }
            HubEvent::Spine(event) => {
                if let Some(MonitoringEvent::Measured { readings, .. }) = monitor.handle_event(&event) {
                    for reading in readings {
                        if reading.measurand == monitoring::Measurand::total_power() {
                            power = reading.value;
                        }
                    }
                }
                guard.handle_event(hub.engine_mut(), &event, now).into_iter().collect()
            }
            HubEvent::Tick => guard.handle_timeout(hub.engine_mut(), now),
            HubEvent::HandshakeFailed { error, .. } => panic!("handshake failed: {error}"),
            _ => Vec::new(),
        };
        for report in reports {
            match report {
                GuardEvent::LimitAccepted { limit, .. } if stage == 0 => {
                    assert_eq!(limit.watts, 3_000.0);
                    stage = 1;
                    let now = hub.now();
                    guard.require_all(Some(LimitWrite::deactivated()), now);
                }
                GuardEvent::LimitAccepted { limit, .. } if stage == 1 && !limit.is_active => {
                    stage = 2;
                }
                GuardEvent::LimitRefused { error, .. } => panic!("refused: {error}"),
                _ => {}
            }
        }
        if stage == 2 && power.is_some() {
            break;
        }
    }
    assert_eq!(stage, 2, "the guard saw both writes accepted");
    assert_eq!(power, Some(1_500.0), "MPC delivered the firmware's power reading");

    wait_for("the firmware to see the release", || {
        LIMITS
            .lock()
            .unwrap()
            .iter()
            .any(|&(state, limited, _)| state == STATE_UNLIMITED_CONTROLLED && limited == 0)
    });
    let limits = LIMITS.lock().unwrap().clone();
    assert!(
        limits.contains(&(STATE_LIMITED, 1, 3_000.0)),
        "the 3 kW limit reached the firmware: {limits:?}"
    );
    assert!(
        PEERS.lock().unwrap().iter().any(|(s, c)| s == &box_ski.to_str().unwrap().to_lowercase() && *c == 1),
        "the peer callback reported the control box"
    );

    hub.shutdown(eebus::ship::ConnectionCloseReason::Unspecific).await;
    unsafe { eebus_device_free(device) };

    // ---- restart: same SKI, the control box still trusted
    LIMITS.lock().unwrap().clear();
    let mut again = std::ptr::null_mut();
    assert_eq!(unsafe { eebus_device_new(&config, &mut again) }, 0);
    let ski_again = unsafe { CStr::from_ptr(eebus_device_ski(again)) }.to_str().unwrap();
    assert_eq!(ski_again, ski, "the identity survives a restart");
    let trust = std::fs::read_to_string(dir.path().join("trust.json")).unwrap();
    assert!(trust.contains(&box_ski.to_str().unwrap().to_lowercase()));
    unsafe { eebus_device_free(again) };
}

#[test]
fn bad_configuration_is_reported() {
    let mut config = std::mem::MaybeUninit::<EebusDeviceConfig>::uninit();
    let mut config = unsafe {
        eebus_device_config_init(config.as_mut_ptr());
        config.assume_init()
    };
    let mut device = std::ptr::null_mut();
    assert_eq!(unsafe { eebus_device_new(&config, &mut device) }, -1);
    assert!(last_error().unwrap().contains("state_dir"));

    let dir = tempfile::tempdir().unwrap();
    let state_dir = CString::new(dir.path().to_str().unwrap()).unwrap();
    let entity = CString::new("Dishwasher").unwrap();
    config.state_dir = state_dir.as_ptr();
    config.entity_type = entity.as_ptr();
    assert_eq!(unsafe { eebus_device_new(&config, &mut device) }, -1);
    assert!(last_error().is_some());
}

fn last_error() -> Option<String> {
    let ptr = eebus_last_error();
    (!ptr.is_null()).then(|| unsafe { CStr::from_ptr(ptr) }.to_string_lossy().into_owned())
}
