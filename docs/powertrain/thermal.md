# Thermomodell und Fahrermodell

## Das Thermomodell

Zwei Wärmespeicher — Block und Öl — mit einem Thermostat dazwischen. Der Block
nimmt die Verbrennungswärme auf, gibt sie über das Kühlmittel an den Kühler und
über die Wandung ans Öl ab; das Öl gibt an die Umgebung ab. Die Kühlerleistung
wächst mit der Fahrgeschwindigkeit.

```
thermal(
    block_mass: 120000,
    oil_mass: 40000,
    block_to_oil: 60,
    radiator: 900,
    oil_to_ambient: 30,
    speed_cooling: 25,
    thermostat_open: (85.0 + units.K0),
    thermostat_full: (100.0 + units.K0),
    ambient: (20.0 + units.K0),
    combustion_heat_fraction: 1.0,
    initial_block_temperature: (20.0 + units.K0),
    initial_oil_temperature: (20.0 + units.K0))
```

Alle zwölf Eingänge sind zugleich unter `thermal.*` in der Registry und damit im
Browser live einstellbar.

**Die Kopplung an die Zylinderwand ist opt-in.** Ohne aktiven Antriebsstrang
rechnet die Verbrennung wie eh und je gegen eine feste Wandtemperatur von 90 °C —
das ist das Verhalten, das jedes unveränderte Motorskript kennt. Erst mit
`set_powertrain(...)` schreibt das Thermomodell die Blocktemperatur in die
Zylinder zurück, und dann startet der Motor kalt.

Wer den Antriebsstrang nutzt und trotzdem warm anfangen will, setzt
`initial_block_temperature` und `initial_oil_temperature` auf 90 °C.

Die Öltemperatur ist das, woran die Adaption ihre `require_warm`-Freigabe hängt
(`docs/powertrain/adaptation.md`), und sie speist die Anlassertemperatur.

## Das Fahrermodell

Der Fahrer ist kein Sprung: Gas und Kupplung folgen dem Eingabegerät mit einer
Zeitkonstante, das Kupplungspedal zusätzlich mit einer Rate.

```
driver(
    pedal_time_constant: 0.024,
    clutch_time_constant: 0.001,
    clutch_pedal_rate: 0.2)
```

Die Vorgabe `0.024 s` reproduziert exakt den alten Bildfilter bei 60 Hz — ohne
Angabe ändert sich also nichts.

## Beides am Antriebsstrang

```
set_powertrain(
    ecu: engine_control_unit(),
    tcu: transmission_control_unit(),
    thermal: thermal(initial_block_temperature: (90.0 + units.K0)),
    driver: driver(pedal_time_constant: 0.05))
```
