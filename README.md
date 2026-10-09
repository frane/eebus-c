# eebus-c

C bindings for [hupe1980/eebus](https://github.com/hupe1980/eebus), the Rust EEBUS stack. For firmware of heat pumps, wallboxes, batteries and inverters that needs to be **§14a EnWG controllable** and is written in C.

[Deutsch](README.de.md)

> **Status: 0.1.0, early.** Device side only. Tested over loopback against the crate's Energy Guard (LPC + MPC) and its `steuerbox` simulator.

## What you get

One header (`include/eebus.h`) and a static or shared library. The device:

- has a SHIP identity (SKI) that survives restarts, pairs with control boxes and energy managers, and is announced via mDNS
- receives power limits (**LPC**, or **LPP** for production) and handles failsafe values, heartbeat and the state machine of the spec. Your firmware gets one callback: *"don't exceed X W"* or *"no limit"*
- reports power and energy (**MPC**) through two function calls

The protocol runs on its own thread. Your code needs no event loop and no Rust.

```c
#include "eebus.h"

static void on_limit(void *user, eebus_state state, int limited, double watts) {
    if (limited) compressor_cap(watts); else compressor_cap(MAX_POWER);
}

int main(void) {
    eebus_device_config c;
    eebus_device_config_init(&c);
    c.state_dir = "/var/lib/eebus";
    c.product = "HeatPump-0815";           /* unique per device */
    c.brand = "ACME"; c.model = "HP-9";
    c.failsafe_watts = 4200;
    c.nominal_max_watts = 9000;
    c.on_limit = on_limit;

    eebus_device *dev;
    if (eebus_device_new(&c, &dev) || eebus_device_start(dev)) {
        fprintf(stderr, "%s\n", eebus_last_error());
        return 1;
    }
    printf("SKI %s\n", eebus_device_ski(dev));   /* print on the label / show in the app */

    for (;;) {
        eebus_device_set_power(dev, measured_watts());
        eebus_device_set_energy(dev, meter_wh(), -1);
        sleep(1);
    }
}
```

A complete example is in [`examples/heat_pump.c`](examples/heat_pump.c).

## Build

You need Rust (rustup) and a C compiler.

```bash
make            # target/release/libeebus_c.{a,so} and target/release/heat_pump
make test       # Rust tests, incl. a real LPC/MPC session over loopback
make install PREFIX=/usr/local
```

Link statically with `libeebus_c.a -lpthread -ldl -lm`.

To cross-compile for embedded Linux, for example aarch64:

```bash
rustup target add aarch64-unknown-linux-gnu
make TARGET=aarch64-unknown-linux-gnu CC=aarch64-linux-gnu-gcc
```

A statically linked binary is about 9 MB (x86_64, stripped). Target: Linux with a network stack. Bare-metal microcontrollers are not supported yet.

## Pairing

On first start the device generates a key in `state_dir` and prints its SKI. A controller is paired in one of two ways:

- **in advance:** `eebus_device_trust(dev, "<controller SKI>")`, for example from an installer app
- **on request:** an unknown controller triggers `on_trust`. Return `EEBUS_TRUST_ACCEPT` (for example after a button press), `EEBUS_TRUST_REFUSE`, or `EEBUS_TRUST_LATER` and call `eebus_device_trust` afterwards.

Trust, key and failsafe values are stored in `state_dir`. Keep this directory: a new key means pairing again.

## Limits

- `on_limit` runs on the library thread. Store the value there and act on it in your own loop.
- Callbacks fire once after start (state `init`, failsafe value) and whenever state or limit changes.
- When the controller goes silent, the failsafe value applies automatically. After the failsafe duration the state becomes `unlimited/autonomous`.
- `eebus_device_interrupt()` overrides an active limit for a reason the spec allows: self-protection, safety, regulatory or uncontrolled loads.

## Scope

| | 0.1 | later |
|---|---|---|
| LPC or LPP, one per device | ✓ | both on one device (battery) |
| MPC power and energy | ✓ | per phase, current, voltage, frequency |
| mDNS, pairing by SKI | ✓ | SHIP Pairing Service (QR secret) |
| Energy manager side (CEM) | – | ✓ |
| EV charging use cases (OPEV, OSCEV, …) | – | ✓ |

## License

MIT. The underlying crate [eebus](https://crates.io/crates/eebus) is MIT/Apache-2.0.
