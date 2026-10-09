/*
 * eebus-c: an EEBUS §14a controllable system for C firmware.
 *
 * A heat pump, wallbox, battery or inverter written in C gets the EEBUS device side:
 * SHIP identity and pairing, a power limit from the grid operator's control box or an
 * energy manager (LPC, or LPP for production), the failsafe behaviour, and power and
 * energy measurements (MPC). The protocol runs on its own thread; the firmware hears
 * about limits through a callback and reports measurements through function calls.
 *
 * Built on the Rust crate `eebus` (https://github.com/hupe1980/eebus).
 *
 * Threading: every callback runs on the library's own thread. Keep callbacks short and
 * do not call eebus_device_free() from inside one. All other functions are thread-safe.
 *
 * Errors: functions returning int return 0 on success and -1 on failure;
 * eebus_last_error() then describes the failure (per calling thread).
 */
#ifndef EEBUS_H
#define EEBUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EEBUS_C_VERSION "0.1.0"

typedef struct eebus_device eebus_device;

/* Which way the limit points: LPC limits consumption, LPP limits production. */
typedef enum {
    EEBUS_LPC = 0,
    EEBUS_LPP = 1
} eebus_direction;

/* The LPC/LPP state machine (LPC/LPP UC TS §2.3.2). */
typedef enum {
    EEBUS_STATE_INIT = 0,                   /* just started, failsafe applies */
    EEBUS_STATE_LIMITED = 1,                /* an active limit applies */
    EEBUS_STATE_UNLIMITED_CONTROLLED = 2,   /* controlled, no limit */
    EEBUS_STATE_FAILSAFE = 3,               /* controller silent, failsafe applies */
    EEBUS_STATE_UNLIMITED_AUTONOMOUS = 4    /* controller gone, device runs on its own */
} eebus_state;

/* Why the device overrides an active limit (eebus_device_interrupt). */
typedef enum {
    EEBUS_REASON_SELF_PROTECTION = 0,
    EEBUS_REASON_SAFETY = 1,
    EEBUS_REASON_REGULATORY = 2,
    EEBUS_REASON_UNCONTROLLED_LOADS = 3
} eebus_reason;

/* Answers of the trust callback. */
#define EEBUS_TRUST_REFUSE 0
#define EEBUS_TRUST_ACCEPT 1
#define EEBUS_TRUST_LATER (-1)   /* decide later with eebus_device_trust() */

/*
 * The limit the device has to follow now.
 *   limited = 0: no limit, run freely (watts is 0).
 *   limited = 1: do not exceed `watts` (consumption for LPC, production for LPP).
 * Called once after start and then whenever the state or the limit changes.
 */
typedef void (*eebus_limit_cb)(void *user, eebus_state state, int limited, double watts);

/*
 * An unknown controller wants to pair. `ski` is its 40-digit hex SKI.
 * Return EEBUS_TRUST_ACCEPT, EEBUS_TRUST_REFUSE or EEBUS_TRUST_LATER.
 * Without this callback, unknown peers stay pending until eebus_device_trust().
 */
typedef int (*eebus_trust_cb)(void *user, const char *ski);

/* A peer connected (1) or went away (0). */
typedef void (*eebus_peer_cb)(void *user, const char *ski, int connected);

typedef struct {
    /* Directory for identity, trust store and failsafe values. Required. Keep it:
     * a new identity means pairing again. */
    const char *state_dir;

    /* Identification. NULL picks a default. */
    const char *vendor;        /* IANA PEN or vendor code, e.g. "46925" */
    const char *product;       /* unique device name, e.g. serial-based: "HeatPump-0815" */
    const char *brand;
    const char *model;
    const char *serial;
    /* SPINE device type: "HeatGenerationSystem", "ChargingStation", "Inverter", ... */
    const char *device_type;
    /* SPINE entity type: "HeatPumpAppliance", "EVSE", "Inverter", "BatterySystem", ... */
    const char *entity_type;

    uint16_t port;             /* SHIP port; 0 = 4712 */
    int mdns;                  /* announce via mDNS (_ship._tcp); recommended 1 */
    const char *announce_ip;   /* address to announce; NULL = detect */

    eebus_direction direction;
    double failsafe_watts;           /* used until a controller sets its own */
    uint32_t failsafe_duration_s;    /* 7200..86400 per spec; 0 = 7200 */
    double nominal_max_watts;        /* most the device can draw/feed; 0 = unknown */
    int mpc;                         /* serve measurements (MPC) */

    eebus_limit_cb on_limit;
    eebus_trust_cb on_trust;
    eebus_peer_cb on_peer;
    void *user;                      /* passed to every callback */
} eebus_device_config;

/* Fills `config` with defaults. Set state_dir and the callbacks afterwards. */
void eebus_device_config_init(eebus_device_config *config);

/* Creates the device and its identity (loaded from state_dir or generated). */
int eebus_device_new(const eebus_device_config *config, eebus_device **out);

/* This device's SKI, 40 hex digits. Valid until eebus_device_free(). */
const char *eebus_device_ski(const eebus_device *device);

/* The SHIP ID, e.g. "i:46925_u:HeatPump-0815". Valid until eebus_device_free(). */
const char *eebus_device_ship_id(const eebus_device *device);

/* Trusts a controller by SKI (hex, spaces allowed) and persists it. Works before and
 * after start; a pending peer is approved immediately. */
int eebus_device_trust(eebus_device *device, const char *ski);

/* Forgets a trusted peer and persists it. */
int eebus_device_untrust(eebus_device *device, const char *ski);

/* Starts listening, announcing and the protocol thread. */
int eebus_device_start(eebus_device *device);

/* MPC: momentary total active power in W (load convention: consumption positive,
 * production negative). */
int eebus_device_set_power(eebus_device *device, double watts);

/* MPC: energy counters in Wh. Pass a negative value to leave one unchanged. */
int eebus_device_set_energy(eebus_device *device, double consumed_wh, double produced_wh);

/* Overrides an active limit for a reason the spec allows (LPC/LPP §2.3.3). */
int eebus_device_interrupt(eebus_device *device, eebus_reason reason);

/* Stops the protocol thread and frees the device. NULL is ignored. */
void eebus_device_free(eebus_device *device);

/* The last error on this thread, or NULL. */
const char *eebus_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* EEBUS_H */
