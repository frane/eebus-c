# eebus-c

C-Bindings für [hupe1980/eebus](https://github.com/hupe1980/eebus), den EEBUS-Stack in Rust. Für Firmware von Wärmepumpen, Wallboxen, Speichern und Wechselrichtern, die **nach §14a EnWG steuerbar** sein muss und in C geschrieben ist.

[English](README.md)

> **Status: 0.1.0, früh.** Nur die Geräteseite. Getestet über Loopback gegen den Energy Guard der Crate (LPC + MPC) und ihren `steuerbox`-Simulator.

## Was du bekommst

Einen Header (`include/eebus.h`) und eine statische oder dynamische Bibliothek. Das Gerät:

- hat eine SHIP-Identität (SKI), die Neustarts übersteht. Es koppelt sich mit Steuerboxen und Energiemanagern und wird per mDNS angekündigt.
- empfängt Leistungslimits (**LPC**, für Erzeugung **LPP**) und übernimmt Failsafe, Heartbeat und die Zustandsmaschine der Spezifikation. Deine Firmware bekommt nur einen Callback: „nicht mehr als X W“ oder „kein Limit“.
- meldet Leistung und Energie (**MPC**) über zwei Funktionsaufrufe.

Das Protokoll läuft in einem eigenen Thread. Dein Code braucht keine Event-Loop und kein Rust.

Ein Beispiel steht in [`README.md`](README.md), ein vollständiges in [`examples/heat_pump.c`](examples/heat_pump.c).

## Bauen

Du brauchst Rust (rustup) und einen C-Compiler.

```bash
make            # target/release/libeebus_c.{a,so} und target/release/heat_pump
make test       # Rust-Tests, inkl. echter LPC/MPC-Sitzung über Loopback
make install PREFIX=/usr/local
```

Statisch linken mit `libeebus_c.a -lpthread -ldl -lm`. Cross-Compile für Embedded Linux: `make TARGET=aarch64-unknown-linux-gnu CC=aarch64-linux-gnu-gcc` (vorher `rustup target add …`).

Ein statisch gelinktes Programm ist etwa 9 MB groß (x86_64, gestrippt). Ziel ist Linux mit Netzwerk. Mikrocontroller ohne Betriebssystem gehen noch nicht.

## Kopplung

Beim ersten Start erzeugt das Gerät einen Schlüssel in `state_dir` und gibt seine SKI aus. Ein Steuergerät wird auf einem von zwei Wegen gekoppelt:

- **vorab:** mit `eebus_device_trust(dev, "<SKI>")`, z. B. aus einer Installateurs-App
- **auf Anfrage:** Ein unbekanntes Steuergerät löst `on_trust` aus. Gib `EEBUS_TRUST_ACCEPT` zurück (z. B. nach Tastendruck), `EEBUS_TRUST_REFUSE`, oder `EEBUS_TRUST_LATER` und rufe später `eebus_device_trust` auf.

Vertrauen, Schlüssel und Failsafe-Werte liegen in `state_dir`. **Dieses Verzeichnis behalten.** Ein neuer Schlüssel heißt: neu koppeln.

## Limits

- `on_limit` läuft im Thread der Bibliothek. Den Wert dort nur merken und in der eigenen Schleife umsetzen.
- Der Callback kommt einmal nach dem Start (Zustand `init` mit dem Failsafe-Wert) und dann bei jeder Änderung von Zustand oder Limit.
- Schweigt das Steuergerät, greift automatisch der Failsafe-Wert. Nach Ablauf der Failsafe-Dauer wechselt der Zustand zu `unlimited/autonomous`.
- `eebus_device_interrupt()` setzt ein aktives Limit außer Kraft, wenn die Spezifikation das erlaubt: Selbstschutz, Sicherheit, gesetzliche Vorgaben oder ungesteuerte Lasten.

## Umfang

| | 0.1 | später |
|---|---|---|
| LPC oder LPP, eins pro Gerät | ✓ | beides auf einem Gerät (Speicher) |
| MPC Leistung und Energie | ✓ | je Phase, Strom, Spannung, Frequenz |
| mDNS, Kopplung per SKI | ✓ | SHIP Pairing Service (QR-Secret) |
| Energiemanager-Seite (CEM) | – | ✓ |
| E-Mobilität (OPEV, OSCEV, …) | – | ✓ |

## Lizenz

MIT. Die zugrunde liegende Crate [eebus](https://crates.io/crates/eebus) steht unter MIT/Apache-2.0.
