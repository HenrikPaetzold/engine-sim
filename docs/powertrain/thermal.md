# Thermomodell und Fahrermodell

## Das Thermomodell

Zwei Wärmespeicher — Block und Öl — mit einem Thermostat dazwischen.

**Der Block** nimmt die Verbrennungswärme auf und gibt sie über das Kühlmittel
an den Kühler und über die Wandung ans Öl ab. Die Kühlerleistung wächst mit der
Fahrgeschwindigkeit.

**Das Öl** hat zwei Zuflüsse und drei Abflüsse:

Richtung | Weg | Parameter
---|---|---
zu | vom Block über die Wandung | `block_to_oil`
zu | **Reibungswärme** aus Kolben und Lagern | `friction.heat_to_oil`, siehe [friction.md](friction.md)
ab | Ölwanne an die Umgebung | `oil_to_ambient`
ab | **Ölkühler**, mit eigenem Thermostat und luftstromabhängig | `oil_cooler`
ab | zurück in den Block, sobald das Öl heißer ist als er | `block_to_oil`, Vorzeichen dreht

Der dritte Abfluss ist der, den man leicht übersieht: `block_to_oil · (T_Block −
T_Öl)` wird negativ, wenn das Öl den Block überholt. Weil der Block über den
Wasserkühler sehr viel Autorität hat und praktisch auf Thermostattemperatur
festliegt, wirkt er dann als Kühlkörper fürs Öl.

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
    oil_cooler: 0.0,
    oil_thermostat_open: (90.0 + units.K0),
    oil_thermostat_full: (105.0 + units.K0),
    initial_block_temperature: (20.0 + units.K0),
    initial_oil_temperature: (20.0 + units.K0))
```

Alle fünfzehn Eingänge sind zugleich unter `thermal.*` in der Registry und damit
im Browser live einstellbar.

**Der Ölkühler ist ab Werk nicht vorhanden** (`oil_cooler: 0`). Sein Thermostat
öffnet später als der des Kühlwassers — 90 statt 85 °C — weil Öl heißer laufen
darf und auch soll, damit Kondenswasser austrägt. Wie heiß es ohne Ölkühler
wird, und warum das Öl bei 200 °C nicht etwa weniger, sondern mehr Reibung
erzeugt, steht in [friction.md](friction.md).

**Die Kopplung an die Zylinderwand ist opt-in.** Ohne aktiven Antriebsstrang
rechnet die Verbrennung wie eh und je gegen eine feste Wandtemperatur von 90 °C —
das ist das Verhalten, das jedes unveränderte Motorskript kennt. Erst mit
`set_powertrain(...)` schreibt das Thermomodell die Blocktemperatur in die
Zylinder zurück, und dann startet der Motor kalt.

Wer den Antriebsstrang nutzt und trotzdem warm anfangen will, setzt
`initial_block_temperature` und `initial_oil_temperature` auf 90 °C.

Die Öltemperatur ist das, woran die Adaption ihre `require_warm`-Freigabe hängt
(`docs/powertrain/adaptation.md`), sie speist die Anlassertemperatur — und seit
der Reibungsstufe steuert sie über die Ölviskosität, wie schwer der Motor
überhaupt zu drehen ist ([friction.md](friction.md)). Damit ist sie kein reiner
Anzeigewert mehr, sondern Teil eines geschlossenen Kreises.

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
