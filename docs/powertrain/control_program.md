# Regelung als Blockschaltbild im Skript

## Warum

Bis hierher parametriert das Skript C++-Regler (ECU, TCU). Eine *neue* Regelung
brauchte C++. Stufe B schließt das: ein `.mr`-Skript beschreibt einen
Blockschaltbild-Graphen, der pro Regeltakt ausgewertet wird.

`.mr` selbst kann das nicht ausführen — Piranha läuft genau einmal beim Laden
und kennt weder `if` noch Schleifen noch Zustand. Das Skript *baut* deshalb den
Graphen; ausgeführt wird er von `control::ControlProgram`.

## Aufbau

```
Skript (einmalig)          Laufzeit (pro Regeltakt)
─────────────────          ────────────────────────
control_block-Knoten  ──►  ControlBlock-Objekte
       │                          │
       ▼                          ▼
ControlProgramNode    ──►  ControlProgram
   .generate()                 .update(dt)
                                  │
              SignalTable ────────┴──────► SignalTable
              (Sensorik)                   (Aktorik)
```

`ScriptedControlUnit : public PowertrainController` füllt die Eingangstabelle
aus `PowertrainState` und `DriverInputs`, ruft `ControlProgram::update(dt)` und
liest die Ausgangstabelle nach `ActuatorCommands`. Damit hängt eine
Skriptregelung an genau derselben Schnittstelle wie ECU und TCU.

## Auswertungsreihenfolge

Beim Laden wird der Graph topologisch sortiert (iterative Tiefensuche mit
Drei-Farben-Markierung). Danach ist ein Takt ein linearer Durchlauf über einen
`int`-Vektor — keine Rekursion, keine Allokation.

Die Deklarationsreihenfolge im Skript spielt keine Rolle; die
Datenabhängigkeiten bestimmen die Reihenfolge.

**Rückkopplungen** brauchen einen `delay`-Block. Jeder andere Zyklus ist ein
Ladefehler mit Nennung der beiden beteiligten Blöcke. `delay` liefert im Takt
*n* den Wert seines Eingangs aus Takt *n−1*; die Übernahme passiert in einem
zweiten Durchlauf (`latch`) nach der Auswertung, damit die Verzögerung genau
einen Takt beträgt, unabhängig von der Sortierung.

## Blöcke

| Block | Operanden | Wirkung |
|---|---|---|
| `signal(channel)` | — | liest eine Messgröße |
| `constant(value)` | — | Festwert; erscheint in der Registry |
| `sum` `product` `minimum` `maximum` | beliebig viele | Verknüpfung |
| `gain(gain, offset)` | 1 | `a·k + b` |
| `clamp(min, max)` | 1 | Begrenzung |
| `curve(curve)` | 1 | `Function`-Kennlinie |
| `lookup(map)` | x, y | `Map2d`-Kennfeld |
| `pid(controller)` | Sollwert, Istwert, Vorsteuerung | vollständiger PID mit Anti-Windup |
| `rate_limit(rise, fall)` | 1 | Gradientenbegrenzung |
| `low_pass(tau)` | 1 | PT1 |
| `select(threshold)` | Bedingung, dann, sonst | Verzweigung ohne `if` in der Sprache |
| `greater_than(band)` | a, b | Vergleich mit Hysterese |
| `latch(threshold)` | Setzen, Rücksetzen | RS-Speicher |
| `integrator(min, max)` | 1 | begrenzter Integrator |
| `timer(threshold)` | Bedingung | Sekunden, solange die Bedingung hält |
| `delay(initial)` | 1 | ein Takt Verzögerung, bricht Zyklen |
| `actuator(channel)` | 1 | schreibt eine Stellgröße |

Die Kanalnamen sind die Felder von `PowertrainState`/`DriverInputs` bzw.
`ActuatorCommands` in `snake_case`; die Tabellen stehen in
`src/powertrain/scripted_control_unit.cpp` und sind per `static_assert` an die
Enums gebunden — ein neues Feld ohne Namen bricht den Build.

## Sicheres Verhalten ohne Programm

Vor jedem Takt werden die Stellgrößen mit unkritischen Werten vorbelegt:
Zündung an, Anlasser aus, Anfettung 1.0, Zielgang = aktueller Gang, Kupplung
unverändert. Ein Programm, das eine Stellgröße nicht bedient, lässt sie damit
in Ruhe — ein leeres Programm ist ein sauberes No-Op.

## Beispiel: Drehzahlbegrenzer, komplett im Skript

```
set_control_program(
    control_program()
        .add_output(
            actuator(
                channel: "ignition_cut",
                a: greater_than(
                    a: signal(channel: "engine_rpm"),
                    b: constant(7000),
                    band: 50)))
        .add_output(
            actuator(
                channel: "throttle_plate",
                a: curve(a: signal(channel: "accelerator"), curve: pedal_map))))
```

## Registry

Benannte Blöcke landen automatisch in der Parameter-Registry und damit in der
Browser-Oberfläche: `constant` als Wert, `gain` als `.gain`/`.offset`, `pid`
als `.kp`/`.ki`/`.kd`, `clamp` als `.min`/`.max`, `rate_limit` als
`.rise`/`.fall`, `low_pass` als `.tau` — jeweils unter `program.<name>`.
Ein Block ohne `name` bleibt unsichtbar.

## Verhältnis zu ECU und TCU

`set_powertrain()` und `set_control_program()` schließen sich aus. Sind beide
gesetzt, gewinnt das Skriptprogramm, und die Adaption bleibt aus — sie lernt in
die ECU/TCU hinein, die dann nicht läuft. Die eingebauten Steuergeräte bleiben
als Referenz und als der Weg mit Adaption bestehen.
