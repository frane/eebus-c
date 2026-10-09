//! C bindings for the [`eebus`] crate: the device side of §14a.
//!
//! One entity plays the LPC (or LPP) Controllable System and, optionally, the MPC
//! Monitored Unit. The protocol runs on a thread of its own with a single-threaded Tokio
//! runtime; the C side receives limits through a callback and reports measurements
//! through plain function calls. See `include/eebus.h` for the contract.

#![allow(clippy::missing_safety_doc)]

use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char, c_int, c_void};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use eebus::cert::{self, CertParams, Identity};
use eebus::model::{
    CmdData, DeviceClassificationManufacturerData, DeviceType, EntityType, FeatureAddress, Role,
    FeatureType, Function,
};
use eebus::runtime::{Hub, HubEvent, Node, TrustStore, TrustedPeer};
use eebus::ship::{ShipId, ShipTxtRecord, Ski};
use eebus::spine::{Engine, LocalDevice, LocalEntity, LocalFeature, Operations};
use eebus::tls::ShipTls;
use eebus::usecases::limitation::{
    self, ControllableSystem, ControllableSystemActor, ControllableSystemBuilder, CsConfig,
    CsFeatures, Direction, EffectiveLimit, LimitationState, RejectReason,
};
use eebus::usecases::monitoring::{Measurand, MonitoredUnit, Quantity};
use eebus::usecases::{lpc, lpp, mpc};

// ---- the C types ------------------------------------------------------------------

pub type LimitCb = Option<unsafe extern "C" fn(*mut c_void, c_int, c_int, f64)>;
pub type TrustCb = Option<unsafe extern "C" fn(*mut c_void, *const c_char) -> c_int>;
pub type PeerCb = Option<unsafe extern "C" fn(*mut c_void, *const c_char, c_int)>;

/// Mirrors `eebus_device_config` in `eebus.h`.
#[repr(C)]
pub struct EebusDeviceConfig {
    pub state_dir: *const c_char,
    pub vendor: *const c_char,
    pub product: *const c_char,
    pub brand: *const c_char,
    pub model: *const c_char,
    pub serial: *const c_char,
    pub device_type: *const c_char,
    pub entity_type: *const c_char,
    pub port: u16,
    pub mdns: c_int,
    pub announce_ip: *const c_char,
    pub direction: c_int,
    pub failsafe_watts: f64,
    pub failsafe_duration_s: u32,
    pub nominal_max_watts: f64,
    pub mpc: c_int,
    pub on_limit: LimitCb,
    pub on_trust: TrustCb,
    pub on_peer: PeerCb,
    pub user: *mut c_void,
}

pub const STATE_INIT: c_int = 0;
pub const STATE_LIMITED: c_int = 1;
pub const STATE_UNLIMITED_CONTROLLED: c_int = 2;
pub const STATE_FAILSAFE: c_int = 3;
pub const STATE_UNLIMITED_AUTONOMOUS: c_int = 4;

pub const TRUST_REFUSE: c_int = 0;
pub const TRUST_ACCEPT: c_int = 1;

const DEFAULT_PORT: u16 = 4712;
const DEFAULT_FAILSAFE_DURATION: u32 = 2 * 3_600;
/// How often the protocol thread looks at commands from the C side.
const POLL: Duration = Duration::from_millis(100);

// Feature addresses on entity [1].
const F_LOAD_CONTROL: u32 = 1;
const F_CONFIGURATION: u32 = 2;
const F_DIAGNOSIS: u32 = 3;
const F_ELECTRICAL: u32 = 4;
const F_DIAGNOSIS_CLIENT: u32 = 5;
const F_MEASUREMENT: u32 = 6;
const F_CLASSIFICATION: u32 = 7;

// ---- errors -----------------------------------------------------------------------

thread_local! {
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

fn set_error(message: impl Into<String>) {
    let message = message.into().replace('\0', " ");
    LAST_ERROR.with(|slot| *slot.borrow_mut() = CString::new(message).ok());
}

fn clear_error() {
    LAST_ERROR.with(|slot| *slot.borrow_mut() = None);
}

type Result<T> = std::result::Result<T, String>;

/// Runs an FFI body: no panic crosses into C, and an `Err` becomes -1 plus a message.
fn guarded(body: impl FnOnce() -> Result<()>) -> c_int {
    match catch_unwind(AssertUnwindSafe(body)) {
        Ok(Ok(())) => {
            clear_error();
            0
        }
        Ok(Err(message)) => {
            set_error(message);
            -1
        }
        Err(_) => {
            set_error("internal error (panic)");
            -1
        }
    }
}

unsafe fn opt_str(ptr: *const c_char) -> Result<Option<String>> {
    if ptr.is_null() {
        return Ok(None);
    }
    let text = unsafe { CStr::from_ptr(ptr) }
        .to_str()
        .map_err(|_| "string is not UTF-8".to_string())?;
    Ok((!text.is_empty()).then(|| text.to_string()))
}

fn parse_ski(text: &str) -> Result<Ski> {
    text.parse::<Ski>()
        .map_err(|error| format!("invalid SKI `{text}`: {error}"))
}

/// Parses a SPINE enumeration by its wire spelling.
fn parse_enum<T: serde::de::DeserializeOwned>(text: &str, what: &str) -> Result<T> {
    serde_json::from_value(serde_json::Value::String(text.to_string()))
        .map_err(|_| format!("unknown {what} `{text}`"))
}

// ---- configuration and persisted state --------------------------------------------

#[derive(Clone)]
struct Callbacks {
    on_limit: LimitCb,
    on_trust: TrustCb,
    on_peer: PeerCb,
    user: *mut c_void,
}

// The user pointer is the caller's business; callbacks only ever run on the protocol
// thread, which is what the header promises.
unsafe impl Send for Callbacks {}

impl Callbacks {
    fn limit(&self, state: c_int, limited: bool, watts: f64) {
        if let Some(cb) = self.on_limit {
            unsafe { cb(self.user, state, c_int::from(limited), watts) };
        }
    }

    /// `Some(true)` accept, `Some(false)` refuse, `None` decide later.
    fn trust(&self, ski: &Ski) -> Option<bool> {
        let cb = self.on_trust?;
        let text = CString::new(ski.to_string()).ok()?;
        match unsafe { cb(self.user, text.as_ptr()) } {
            TRUST_ACCEPT => Some(true),
            TRUST_REFUSE => Some(false),
            _ => None,
        }
    }

    fn peer(&self, ski: &Ski, connected: bool) {
        if let Some(cb) = self.on_peer
            && let Ok(text) = CString::new(ski.to_string())
        {
            unsafe { cb(self.user, text.as_ptr(), c_int::from(connected)) };
        }
    }
}

struct Settings {
    vendor: String,
    product: String,
    brand: String,
    model: String,
    serial: Option<String>,
    device_type: DeviceType,
    device_type_name: String,
    entity_type: EntityType,
    port: u16,
    mdns: bool,
    announce_ip: Option<std::net::IpAddr>,
    direction: Direction,
    failsafe_watts: f64,
    failsafe_duration: Duration,
    nominal_max: Option<f64>,
    mpc: bool,
}

impl Settings {
    unsafe fn from_c(config: &EebusDeviceConfig) -> Result<(PathBuf, Self)> {
        let state_dir = unsafe { opt_str(config.state_dir) }?
            .ok_or("state_dir is required")?;
        let direction = match config.direction {
            0 => lpc::DIRECTION,
            1 => lpp::DIRECTION,
            other => return Err(format!("unknown direction {other}")),
        };
        let device_type_name = unsafe { opt_str(config.device_type) }?
            .unwrap_or_else(|| "HeatGenerationSystem".into());
        let entity_name =
            unsafe { opt_str(config.entity_type) }?.unwrap_or_else(|| "HeatPumpAppliance".into());
        let announce_ip = match unsafe { opt_str(config.announce_ip) }? {
            Some(ip) => Some(ip.parse().map_err(|_| format!("invalid announce_ip `{ip}`"))?),
            None => None,
        };
        if !(config.failsafe_watts.is_finite() && config.failsafe_watts >= 0.0) {
            return Err("failsafe_watts must be a non-negative number".into());
        }
        let settings = Self {
            vendor: unsafe { opt_str(config.vendor) }?.unwrap_or_else(|| "46925".into()),
            product: unsafe { opt_str(config.product) }?.unwrap_or_else(|| "Device-1".into()),
            brand: unsafe { opt_str(config.brand) }?.unwrap_or_else(|| "eebus-c".into()),
            model: unsafe { opt_str(config.model) }?.unwrap_or_else(|| "eebus-c".into()),
            serial: unsafe { opt_str(config.serial) }?,
            device_type: parse_enum(&device_type_name, "device_type")?,
            device_type_name,
            entity_type: parse_enum(&entity_name, "entity_type")?,
            port: if config.port == 0 { DEFAULT_PORT } else { config.port },
            mdns: config.mdns != 0,
            announce_ip,
            direction,
            failsafe_watts: config.failsafe_watts,
            failsafe_duration: Duration::from_secs(u64::from(if config.failsafe_duration_s == 0 {
                DEFAULT_FAILSAFE_DURATION
            } else {
                config.failsafe_duration_s
            })),
            nominal_max: (config.nominal_max_watts > 0.0).then_some(config.nominal_max_watts),
            mpc: config.mpc != 0,
        };
        let use_case = settings.limitation_use_case();
        if !use_case.permits_entity(&settings.entity_type) {
            return Err(format!(
                "entity_type `{entity_name}` cannot play the {} Controllable System",
                use_case.name
            ));
        }
        if settings.mpc && !mpc::MONITORED_UNIT.permits_entity(&settings.entity_type) {
            return Err(format!("entity_type `{entity_name}` cannot play the MPC Monitored Unit"));
        }
        Ok((PathBuf::from(state_dir), settings))
    }

    fn limitation_use_case(&self) -> &'static eebus::usecases::descriptor::UseCaseDescriptor {
        if self.direction == lpp::DIRECTION {
            &lpp::CONTROLLABLE_SYSTEM
        } else {
            &lpc::CONTROLLABLE_SYSTEM
        }
    }
}

/// The files a device keeps: its key, whom it trusts, and the failsafe values a
/// controller last set (LPC/LPP require them to survive a restart).
struct Store {
    dir: PathBuf,
}

impl Store {
    fn open(dir: &Path) -> Result<Self> {
        std::fs::create_dir_all(dir)
            .map_err(|error| format!("cannot create {}: {error}", dir.display()))?;
        Ok(Self {
            dir: dir.to_path_buf(),
        })
    }

    fn identity(&self, ship_id: &str) -> Result<Identity> {
        let path = self.dir.join("identity.key.pem");
        if let Ok(pem) = std::fs::read_to_string(&path) {
            let key = cert::key_from_pem(&pem).map_err(|e| format!("{}: {e}", path.display()))?;
            return cert::self_signed_with(CertParams::new(ship_id), key).map_err(|e| e.to_string());
        }
        let identity = cert::self_signed(CertParams::new(ship_id)).map_err(|e| e.to_string())?;
        write_private(&path, identity.key_pem().as_bytes())
            .map_err(|e| format!("{}: {e}", path.display()))?;
        Ok(identity)
    }

    fn trust(&self) -> TrustStore {
        std::fs::read_to_string(self.dir.join("trust.json"))
            .ok()
            .and_then(|json| TrustStore::from_json(&json).ok())
            .unwrap_or_default()
    }

    fn save_trust(&self, trust: &TrustStore) -> Result<()> {
        let json = trust.to_json().map_err(|e| e.to_string())?;
        write_atomic(&self.dir.join("trust.json"), json.as_bytes()).map_err(|e| e.to_string())
    }

    fn failsafe(&self) -> Option<(f64, Duration)> {
        let json = std::fs::read_to_string(self.dir.join("failsafe.json")).ok()?;
        let value: serde_json::Value = serde_json::from_str(&json).ok()?;
        Some((
            value.get("watts")?.as_f64()?,
            Duration::from_secs(value.get("duration_s")?.as_u64()?),
        ))
    }

    fn save_failsafe(&self, watts: f64, duration: Duration) {
        let json = serde_json::json!({ "watts": watts, "duration_s": duration.as_secs() });
        let _ = write_atomic(&self.dir.join("failsafe.json"), json.to_string().as_bytes());
    }
}

fn write_atomic(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let tmp = path.with_extension("tmp");
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(tmp, path)
}

#[cfg(unix)]
fn write_private(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    use std::os::unix::fs::OpenOptionsExt;
    std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .mode(0o600)
        .open(path)?
        .write_all(bytes)
}

#[cfg(not(unix))]
fn write_private(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    std::fs::write(path, bytes)
}

// ---- the device -------------------------------------------------------------------

#[derive(Default, Clone, Copy, PartialEq)]
struct Readings {
    power: Option<f64>,
    consumed: Option<f64>,
    produced: Option<f64>,
}

enum Command {
    Approve(Ski),
    Forget(Ski),
    Interrupt(RejectReason),
    Measure,
    Stop,
}

/// What `eebus_device_start` hands to the protocol thread.
struct Setup {
    settings: Settings,
    store: Store,
    identity: Identity,
    ship_id: ShipId,
    callbacks: Callbacks,
}

pub struct EebusDevice {
    ski: CString,
    ship_id: CString,
    trust: TrustStore,
    store_dir: PathBuf,
    readings: Arc<Mutex<Readings>>,
    setup: Option<Setup>,
    commands: Option<Sender<Command>>,
    thread: Option<JoinHandle<()>>,
}

impl EebusDevice {
    fn send(&self, command: Command) {
        if let Some(tx) = &self.commands {
            let _ = tx.send(command);
        }
    }

    fn persist_trust(&self) -> Result<()> {
        Store {
            dir: self.store_dir.clone(),
        }
        .save_trust(&self.trust)
    }
}

/// The SPINE device: one entity with LPC/LPP, and MPC when asked for.
struct Model {
    engine: Engine,
    actor: ControllableSystemBuilder,
    unit: Option<(MonitoredUnit, FeatureAddress, FeatureAddress)>,
}

fn build_model(settings: &Settings, failsafe: (f64, Duration)) -> Result<Model> {
    let mut device = LocalDevice::new(
        &format!("i:{}", settings.vendor),
        &settings.product,
        settings.device_type.clone(),
    )
    .map_err(|e| format!("invalid device address: {e}"))?;

    let unit = settings.mpc.then(|| {
        let mut unit = MonitoredUnit::new(1).with(Measurand::total_power());
        unit.add(Measurand::total(Quantity::EnergyConsumed));
        unit.add(Measurand::total(Quantity::EnergyProduced));
        unit
    });

    // One ElectricalConnection feature for both use cases: LPC's characteristics
    // (scenario 4) and MPC's descriptions. Peers look for the feature by type.
    let mut electrical = LocalFeature::new(F_ELECTRICAL, FeatureType::ElectricalConnection, Role::Server)
        .with_function(
            Function::ElectricalConnectionCharacteristicListData,
            Operations::read(),
        );
    if unit.is_some() {
        electrical = electrical
            .with_function(
                Function::ElectricalConnectionDescriptionListData,
                Operations::read(),
            )
            .with_function(
                Function::ElectricalConnectionParameterDescriptionListData,
                Operations::read(),
            );
    }

    let mut entity = LocalEntity::new([1], settings.entity_type.clone())
        .with_feature(limitation::load_control_feature(F_LOAD_CONTROL))
        .with_feature(limitation::device_configuration_feature(F_CONFIGURATION))
        .with_feature(limitation::device_diagnosis_feature(F_DIAGNOSIS))
        .with_feature(limitation::device_diagnosis_client_feature(F_DIAGNOSIS_CLIENT))
        .with_feature(electrical)
        .with_feature(
            LocalFeature::new(F_CLASSIFICATION, FeatureType::DeviceClassification, Role::Server)
                .with_function(
                    Function::DeviceClassificationManufacturerData,
                    Operations::read(),
                ),
        );
    if let Some(unit) = &unit {
        entity = entity.with_feature(unit.measurement_feature(F_MEASUREMENT));
    }
    device
        .add_entity(entity)
        .map_err(|e| format!("cannot add entity: {e}"))?;

    let at = |feature| device.address_of(&[1], feature);
    let features = CsFeatures {
        load_control: at(F_LOAD_CONTROL),
        device_configuration: at(F_CONFIGURATION),
        device_diagnosis: at(F_DIAGNOSIS),
        device_diagnosis_client: at(F_DIAGNOSIS_CLIENT),
    };
    let electrical = at(F_ELECTRICAL);
    let measurement = at(F_MEASUREMENT);
    let classification = at(F_CLASSIFICATION);

    let mut engine = Engine::new(device);
    engine.add_use_case_scenarios([1], 1, settings.limitation_use_case(), &[1, 2, 3, 4]);
    if unit.is_some() {
        engine.add_use_case_scenarios([1], 1, &mpc::MONITORED_UNIT, &[1, 2]);
    }

    if let Some(feature) = engine.device_mut().resolve_mut(&classification) {
        let _ = feature.set_data(CmdData::DeviceClassificationManufacturerData(
            DeviceClassificationManufacturerData {
                device_name: Some(settings.model.as_str().into()),
                device_code: Some(settings.model.as_str().into()),
                brand_name: Some(settings.brand.as_str().into()),
                vendor_name: Some(settings.brand.as_str().into()),
                vendor_code: Some(settings.vendor.as_str().into()),
                serial_number: settings.serial.as_deref().map(Into::into),
                ..Default::default()
            },
        ));
    }

    let mut config = CsConfig::new(failsafe.0, failsafe.1);
    if let Some(max) = settings.nominal_max {
        config = config.with_nominal_max(max);
    }
    let actor = ControllableSystemActor::builder(
        ControllableSystem::new(config, Duration::ZERO),
        settings.direction,
        features,
    )
    .with_electrical_connection(electrical.clone());

    let unit = unit.map(|unit| {
        unit.publish(&mut engine, &electrical, &measurement);
        (unit, electrical, measurement)
    });
    Ok(Model {
        engine,
        actor,
        unit,
    })
}

fn state_code(state: LimitationState) -> c_int {
    match state {
        LimitationState::Init => STATE_INIT,
        LimitationState::Limited => STATE_LIMITED,
        LimitationState::UnlimitedControlled => STATE_UNLIMITED_CONTROLLED,
        LimitationState::FailsafeState => STATE_FAILSAFE,
        LimitationState::UnlimitedAutonomous => STATE_UNLIMITED_AUTONOMOUS,
        #[allow(unreachable_patterns)]
        _ => STATE_INIT,
    }
}

/// A TXT value as SHIP allows it: no whitespace, `;` or control characters, at
/// most 32 characters. SPINE keeps the full spelling; only the announcement is shortened.
fn txt_value(text: &str) -> String {
    text.chars()
        .map(|ch| if ch == ';' || ch.is_control() || ch.is_whitespace() { '-' } else { ch })
        .take(32)
        .collect()
}

/// Addresses to announce: the configured one, or the one the default route leaves by.
fn announce_addresses(configured: Option<std::net::IpAddr>) -> Vec<std::net::IpAddr> {
    if let Some(ip) = configured {
        return vec![ip];
    }
    for target in ["224.0.0.251:5353", "192.0.2.1:9"] {
        if let Ok(socket) = std::net::UdpSocket::bind("0.0.0.0:0")
            && socket.connect(target).is_ok()
            && let Ok(local) = socket.local_addr()
            && !local.ip().is_unspecified()
        {
            return vec![local.ip()];
        }
    }
    vec![std::net::IpAddr::from([127, 0, 0, 1])]
}

// ---- the protocol thread ----------------------------------------------------------

async fn run(
    setup: Setup,
    trust: TrustStore,
    readings: Arc<Mutex<Readings>>,
    commands: Receiver<Command>,
    started: Sender<Result<()>>,
) {
    let Setup {
        settings,
        store,
        identity,
        ship_id,
        callbacks,
    } = setup;

    let failsafe = store
        .failsafe()
        .unwrap_or((settings.failsafe_watts, settings.failsafe_duration));
    let model = match build_model(&settings, failsafe) {
        Ok(model) => model,
        Err(error) => {
            let _ = started.send(Err(error));
            return;
        }
    };
    let ski = identity.ski;
    let node = Node::new(ship_id.as_str(), ShipTls::new(identity), trust.clone());
    let mut hub = Hub::new(node, model.engine);
    let bound = match hub.listen(("0.0.0.0", settings.port)).await {
        Ok(bound) => bound,
        Err(error) => {
            let _ = started.send(Err(format!("cannot listen on port {}: {error}", settings.port)));
            return;
        }
    };

    let _mdns = if settings.mdns {
        let mut record = ShipTxtRecord::new(ship_id.clone(), ski)
            .with_brand(&txt_value(&settings.brand))
            .with_model(&txt_value(&settings.model))
            .with_device_type(&txt_value(&settings.device_type_name));
        if let Some(serial) = &settings.serial {
            record = record.with_serial(&txt_value(serial));
        }
        let announced = eebus::mdns::Mdns::new().and_then(|mut mdns| {
            mdns.announce(
                ship_id.as_str(),
                &record,
                bound.port(),
                &announce_addresses(settings.announce_ip),
            )?;
            Ok(mdns)
        });
        match announced {
            Ok(mdns) => {
                // Finding trusted peers again after a restart is the hub's business.
                let _ = hub.browse(&mdns);
                Some(mdns)
            }
            Err(error) => {
                let _ = started.send(Err(format!("mDNS: {error}")));
                return;
            }
        }
    } else {
        None
    };

    let now = hub.now();
    let mut actor = model.actor.install(hub.engine_mut(), now);
    let mut unit = model.unit;
    let mut last_readings = Readings::default();
    let mut reported: Option<(c_int, bool, f64)> = None;
    let mut saved_failsafe = failsafe;
    let _ = started.send(Ok(()));

    let report = |actor: &ControllableSystemActor, reported: &mut Option<(c_int, bool, f64)>| {
        let system = actor.system();
        let (limited, watts) = match system.effective_limit() {
            EffectiveLimit::None => (false, 0.0),
            EffectiveLimit::Active(w) | EffectiveLimit::Failsafe(w) => (true, w),
            #[allow(unreachable_patterns)]
            _ => (false, 0.0),
        };
        let now = (state_code(system.state()), limited, watts);
        if *reported != Some(now) {
            *reported = Some(now);
            callbacks.limit(now.0, now.1, now.2);
        }
    };
    report(&actor, &mut reported);

    loop {
        // Commands first: `Hub::next` is not cancel-safe, so the loop wakes on a short
        // deadline instead of racing it against a channel.
        let mut stop = false;
        while let Ok(command) = commands.try_recv() {
            let now = hub.now();
            match command {
                Command::Approve(peer) => hub.approve(peer),
                Command::Forget(peer) => {
                    hub.forget_peer(&peer);
                    hub.close(&peer, eebus::ship::ConnectionCloseReason::Unspecific);
                }
                Command::Interrupt(reason) => {
                    actor.interrupt(hub.engine_mut(), reason, now);
                }
                Command::Measure => {}
                Command::Stop => stop = true,
            }
        }
        if stop {
            break;
        }

        let current = readings.lock().map(|r| *r).unwrap_or_default();
        if current != last_readings
            && let Some((unit, _, measurement)) = unit.as_mut()
        {
            let pairs = [
                (Measurand::total_power(), current.power),
                (Measurand::total(Quantity::EnergyConsumed), current.consumed),
                (Measurand::total(Quantity::EnergyProduced), current.produced),
            ];
            for (measurand, value) in pairs {
                if let Some(value) = value {
                    unit.set(&measurand, value);
                }
            }
            let now = hub.now();
            unit.notify(hub.engine_mut(), measurement, now);
            last_readings = current;
        }

        hub.wake_at(actor.poll_timeout());
        hub.wake_at(hub.now() + POLL);
        let now = hub.now();
        let event = match hub.next().await {
            Ok(event) => event,
            Err(_) => break,
        };
        match event {
            HubEvent::TrustRequested { peer, .. } => match callbacks.trust(&peer.ski) {
                Some(true) => {
                    hub.approve(peer.ski);
                    trust.remember(TrustedPeer::new(peer.ski));
                    let _ = store.save_trust(&trust);
                }
                Some(false) => hub.refuse(peer.ski),
                None => {}
            },
            HubEvent::Connected { ski, .. } => {
                let _ = store.save_trust(&trust);
                callbacks.peer(&ski, true);
            }
            HubEvent::Disconnected { ski, .. } => callbacks.peer(&ski, false),
            HubEvent::PeerKeysUpdated { .. } => {
                let _ = store.save_trust(&trust);
            }
            HubEvent::Spine(event) => {
                actor.handle_event(hub.engine_mut(), &event, now);
            }
            HubEvent::Tick => {
                actor.handle_timeout(hub.engine_mut(), now);
            }
            _ => {}
        }

        let config = actor.system().config();
        let failsafe = (config.failsafe_watts, config.failsafe_duration);
        if failsafe != saved_failsafe {
            store.save_failsafe(failsafe.0, failsafe.1);
            saved_failsafe = failsafe;
        }
        report(&actor, &mut reported);
    }

    hub.shutdown(eebus::ship::ConnectionCloseReason::Unspecific)
        .await;
}

// ---- the C API --------------------------------------------------------------------

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_config_init(config: *mut EebusDeviceConfig) {
    if config.is_null() {
        return;
    }
    unsafe {
        config.write(EebusDeviceConfig {
            state_dir: std::ptr::null(),
            vendor: std::ptr::null(),
            product: std::ptr::null(),
            brand: std::ptr::null(),
            model: std::ptr::null(),
            serial: std::ptr::null(),
            device_type: std::ptr::null(),
            entity_type: std::ptr::null(),
            port: DEFAULT_PORT,
            mdns: 1,
            announce_ip: std::ptr::null(),
            direction: 0,
            failsafe_watts: 4_200.0,
            failsafe_duration_s: DEFAULT_FAILSAFE_DURATION,
            nominal_max_watts: 0.0,
            mpc: 1,
            on_limit: None,
            on_trust: None,
            on_peer: None,
            user: std::ptr::null_mut(),
        });
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_new(
    config: *const EebusDeviceConfig,
    out: *mut *mut EebusDevice,
) -> c_int {
    guarded(|| {
        if config.is_null() || out.is_null() {
            return Err("config and out must not be NULL".into());
        }
        let config = unsafe { &*config };
        let (dir, settings) = unsafe { Settings::from_c(config) }?;
        let store = Store::open(&dir)?;
        let ship_id = ShipId::new(&settings.vendor, &settings.product);
        let identity = store.identity(ship_id.as_str())?;
        let trust = store.trust();
        let device = EebusDevice {
            ski: CString::new(identity.ski.to_string()).map_err(|e| e.to_string())?,
            ship_id: CString::new(ship_id.as_str()).map_err(|e| e.to_string())?,
            trust,
            store_dir: dir,
            readings: Arc::new(Mutex::new(Readings::default())),
            setup: Some(Setup {
                settings,
                store,
                identity,
                ship_id,
                callbacks: Callbacks {
                    on_limit: config.on_limit,
                    on_trust: config.on_trust,
                    on_peer: config.on_peer,
                    user: config.user,
                },
            }),
            commands: None,
            thread: None,
        };
        unsafe { *out = Box::into_raw(Box::new(device)) };
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_ski(device: *const EebusDevice) -> *const c_char {
    match unsafe { device.as_ref() } {
        Some(device) => device.ski.as_ptr(),
        None => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_ship_id(device: *const EebusDevice) -> *const c_char {
    match unsafe { device.as_ref() } {
        Some(device) => device.ship_id.as_ptr(),
        None => std::ptr::null(),
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_trust(device: *mut EebusDevice, ski: *const c_char) -> c_int {
    guarded(|| {
        let device = unsafe { device.as_mut() }.ok_or("device is NULL")?;
        let ski = parse_ski(&unsafe { opt_str(ski) }?.ok_or("ski is NULL")?)?;
        device.trust.remember(TrustedPeer::new(ski));
        device.persist_trust()?;
        device.send(Command::Approve(ski));
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_untrust(
    device: *mut EebusDevice,
    ski: *const c_char,
) -> c_int {
    guarded(|| {
        let device = unsafe { device.as_mut() }.ok_or("device is NULL")?;
        let ski = parse_ski(&unsafe { opt_str(ski) }?.ok_or("ski is NULL")?)?;
        device.trust.forget(&ski);
        device.persist_trust()?;
        device.send(Command::Forget(ski));
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_start(device: *mut EebusDevice) -> c_int {
    guarded(|| {
        let device = unsafe { device.as_mut() }.ok_or("device is NULL")?;
        let setup = device.setup.take().ok_or("already started")?;
        let (tx, rx) = mpsc::channel();
        let (started_tx, started_rx) = mpsc::channel();
        let trust = device.trust.clone();
        let readings = device.readings.clone();
        let thread = std::thread::Builder::new()
            .name("eebus".into())
            .spawn(move || {
                let runtime = match tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                {
                    Ok(runtime) => runtime,
                    Err(error) => {
                        let _ = started_tx.send(Err(format!("runtime: {error}")));
                        return;
                    }
                };
                runtime.block_on(run(setup, trust, readings, rx, started_tx));
            })
            .map_err(|e| format!("cannot spawn thread: {e}"))?;
        match started_rx.recv() {
            Ok(Ok(())) => {
                device.commands = Some(tx);
                device.thread = Some(thread);
                Ok(())
            }
            Ok(Err(error)) => {
                let _ = thread.join();
                Err(error)
            }
            Err(_) => {
                let _ = thread.join();
                Err("the protocol thread failed to start".into())
            }
        }
    })
}

fn measure(device: *mut EebusDevice, update: impl FnOnce(&mut Readings)) -> c_int {
    guarded(|| {
        let device = unsafe { device.as_mut() }.ok_or("device is NULL")?;
        let mut readings = device.readings.lock().map_err(|_| "poisoned")?;
        update(&mut readings);
        drop(readings);
        device.send(Command::Measure);
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_set_power(device: *mut EebusDevice, watts: f64) -> c_int {
    if !watts.is_finite() {
        set_error("watts must be finite");
        return -1;
    }
    measure(device, |r| r.power = Some(watts))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_set_energy(
    device: *mut EebusDevice,
    consumed_wh: f64,
    produced_wh: f64,
) -> c_int {
    measure(device, |r| {
        if consumed_wh.is_finite() && consumed_wh >= 0.0 {
            r.consumed = Some(consumed_wh);
        }
        if produced_wh.is_finite() && produced_wh >= 0.0 {
            r.produced = Some(produced_wh);
        }
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_interrupt(device: *mut EebusDevice, reason: c_int) -> c_int {
    guarded(|| {
        let device = unsafe { device.as_mut() }.ok_or("device is NULL")?;
        let reason = match reason {
            0 => RejectReason::SelfProtection,
            1 => RejectReason::SafetyRelated,
            2 => RejectReason::Regulatory,
            3 => RejectReason::UncontrolledLoads,
            other => return Err(format!("unknown reason {other}")),
        };
        if device.commands.is_none() {
            return Err("not started".into());
        }
        device.send(Command::Interrupt(reason));
        Ok(())
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn eebus_device_free(device: *mut EebusDevice) {
    if device.is_null() {
        return;
    }
    let _ = catch_unwind(AssertUnwindSafe(|| {
        let mut device = unsafe { Box::from_raw(device) };
        device.send(Command::Stop);
        if let Some(thread) = device.thread.take() {
            let _ = thread.join();
        }
    }));
}

#[unsafe(no_mangle)]
pub extern "C" fn eebus_last_error() -> *const c_char {
    LAST_ERROR.with(|slot| {
        slot.borrow()
            .as_ref()
            .map_or(std::ptr::null(), |message| message.as_ptr())
    })
}
