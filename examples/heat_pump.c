/*
 * A heat pump in C that follows §14a limits.
 *
 *   ./heat_pump [state-dir] [controller-SKI]
 *
 * It prints its SKI, trusts the given controller (or asks on the terminal),
 * applies every limit it is sent and reports its power draw once a second.
 * Stop it with Ctrl-C.
 */
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "eebus.h"

static volatile sig_atomic_t running = 1;
static volatile double max_watts = 4200.0;   /* failsafe until told otherwise */

static void on_signal(int sig) { (void)sig; running = 0; }

static const char *state_name(eebus_state s) {
    switch (s) {
    case EEBUS_STATE_INIT: return "init";
    case EEBUS_STATE_LIMITED: return "limited";
    case EEBUS_STATE_UNLIMITED_CONTROLLED: return "unlimited/controlled";
    case EEBUS_STATE_FAILSAFE: return "failsafe";
    case EEBUS_STATE_UNLIMITED_AUTONOMOUS: return "unlimited/autonomous";
    }
    return "?";
}

/* Runs on the eebus thread: just record what to do, the main loop acts on it. */
static void on_limit(void *user, eebus_state state, int limited, double watts) {
    (void)user;
    max_watts = limited ? watts : 9000.0;
    if (limited)
        printf("[pump] %s: limit %.0f W\n", state_name(state), watts);
    else
        printf("[pump] %s: no limit\n", state_name(state));
    fflush(stdout);
}

static int on_trust(void *user, const char *ski) {
    (void)user;
    printf("[pump] %s wants to pair. Accepting.\n", ski);
    fflush(stdout);
    return EEBUS_TRUST_ACCEPT;   /* a real device asks the installer here */
}

static void on_peer(void *user, const char *ski, int connected) {
    (void)user;
    printf("[pump] %s %s\n", ski, connected ? "connected" : "disconnected");
    fflush(stdout);
}

int main(int argc, char **argv) {
    eebus_device_config config;
    eebus_device_config_init(&config);
    config.state_dir = argc > 1 ? argv[1] : ".eebus-c-heatpump";
    config.product = "HeatPump-C1";
    config.brand = "Example";
    config.model = "C Heat Pump";
    config.serial = "C1-0001";
    config.device_type = "HeatGenerationSystem";
    config.entity_type = "HeatPumpAppliance";
    config.failsafe_watts = 4200.0;
    config.nominal_max_watts = 9000.0;
    config.on_limit = on_limit;
    config.on_trust = argc > 2 ? NULL : on_trust;
    config.on_peer = on_peer;

    eebus_device *device;
    if (eebus_device_new(&config, &device) != 0) {
        fprintf(stderr, "eebus_device_new: %s\n", eebus_last_error());
        return 1;
    }
    printf("SHIP ID %s\nSKI     %s\n", eebus_device_ship_id(device), eebus_device_ski(device));

    if (argc > 2 && eebus_device_trust(device, argv[2]) != 0) {
        fprintf(stderr, "eebus_device_trust: %s\n", eebus_last_error());
        eebus_device_free(device);
        return 1;
    }
    if (eebus_device_start(device) != 0) {
        fprintf(stderr, "eebus_device_start: %s\n", eebus_last_error());
        eebus_device_free(device);
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    double energy_wh = 0.0;
    while (running) {
        double demand = 6000.0;                       /* what the pump would like */
        double draw = demand < max_watts ? demand : max_watts;
        energy_wh += draw / 3600.0;
        eebus_device_set_power(device, draw);
        eebus_device_set_energy(device, energy_wh, -1);
        sleep(1);
    }

    eebus_device_free(device);
    return 0;
}
