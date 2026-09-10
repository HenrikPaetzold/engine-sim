# Die Steuerungsebene

Eine skriptbare Regelungsschicht über der bestehenden Motorsimulation: ein
Motorsteuergerät, ein Getriebesteuergerät, eine Adaption die im Fahrbetrieb
lernt, ein Blockdiagramm-Interpreter für eigene Regler, eine Parameter-Registry
und eine Weboberfläche, die alles davon live zeigt und ändern kann.

**Ab Werk ist nichts davon an.** Ohne `set_powertrain(...)` oder
`set_control_program(...)` im Skript verhält sich die Simulation wie vorher.

---

## Wo anfangen

Wenn du wissen willst … | dann lies
---|---
wie ein Skript zu einer laufenden Regelung wird | diese Datei, Abschnitt **Der eine Weg**
wie man einen eigenen Regler baut, ohne C++ anzufassen | [control_program.md](control_program.md)
wie das Steuergerät im Fahren lernt | [adaptation.md](adaptation.md)
wie man im Browser dreht und was man dort sieht | [calibration_ui.md](calibration_ui.md)
wie eine Stellgröße heißt — dreimal | [parameter_map.md](parameter_map.md)
wie der Wählhebel und seine Sperren funktionieren | [selector_gate.md](selector_gate.md)
wie Kupplung, Wandler und Fahrzeug aneinanderhängen | [driveline.md](driveline.md)
wie Block und Öl warm werden, und wer den Fahrer filtert | [thermal.md](thermal.md)
wie der Anlasser bedatet wird | [starter.md](starter.md)
wie man das Ganze unter Linux baut | [building-on-linux.md](building-on-linux.md)

---

## Die Karte

Was | Wo es lebt | Erklärt in
---|---|---
**Motorsteuergerät** — Pedal zu Momentenwunsch zu Klappe, Leerlauf, Begrenzer, Zündwinkel | `include/powertrain/engine_control_unit.h`, `src/powertrain/engine_control_unit.cpp` | —
dessen Registry-Deklarationen | `src/powertrain/engine_control_unit_parameters.cpp` | [calibration_ui.md](calibration_ui.md)
**Getriebesteuergerät** — Schaltentscheidung, Schaltablauf, Kupplungsdrücke | `include/powertrain/transmission_control_unit.h`, `src/powertrain/transmission_control_unit.cpp` | —
dessen Vorgabekennfelder und Getriebe-Handschlag | `src/powertrain/transmission_control_unit_defaults.cpp` | —
dessen Registry-Deklarationen | `src/powertrain/transmission_control_unit_parameters.cpp` | [calibration_ui.md](calibration_ui.md)
**Wählhebel** — frei definierbare Gasse mit Sperren | `src/powertrain/selector_gate.cpp`, plus `positionAllowed`/`resolvePosition` in der TCU | [selector_gate.md](selector_gate.md)
**Adaption** — Drosselklappenkennfeld, Leerlauftrimm, Lambdatrimm, Schaltlernen | `src/adaptation/adaptation_manager.cpp` | [adaptation.md](adaptation.md)
**Streckenmodell** — rekursive kleinste Quadrate | `src/adaptation/rls_estimator.cpp` | [adaptation.md](adaptation.md)
**Regelungsbausteine** — PID, Kennfeld, Hysterese, Ratenbegrenzer, ILC | `include/control/`, `src/control/` | [control_program.md](control_program.md)
**Blockprogramm** — der Interpreter für Skriptregler | `src/control/control_program.cpp`, `src/control/control_block.cpp` | [control_program.md](control_program.md)
**Skript-Regeleinheit** — hängt ein Blockprogramm als Regler ein | `src/powertrain/scripted_control_unit.cpp` | [control_program.md](control_program.md)
**Parameter-Registry** — jede Stellgröße als typisierter Zeiger, mit Grenzen und Adaptiv-Freigabe | `src/config/parameter_registry.cpp` | [calibration_ui.md](calibration_ui.md)
**Webserver** — Schema, Live-Zustand, Schreibbefehle | `src/config/config_server.cpp` | [calibration_ui.md](calibration_ui.md)
**Oberfläche** | `assets/config_ui/index.html` | [calibration_ui.md](calibration_ui.md)
**Oszilloskop und Schaltrekorder** | `src/config/channel_recorder.cpp`, `src/config/shift_recorder.cpp` | [calibration_ui.md](calibration_ui.md)
**Fahrmodi** — benannte Parametersätze | `src/config/drive_mode.cpp` | [calibration_ui.md](calibration_ui.md)
**Verdrahtung zur Simulation** — Zustand lesen, Befehle anwenden | `src/powertrain_system.cpp` | —
**Aufbau beim Laden** | `src/powertrain_bootstrap.cpp` | diese Datei, unten
**Thermomodell und Fahrermodell** | `src/thermal_model.cpp`, `src/powertrain_system.cpp` | [thermal.md](thermal.md)
**Antriebsstrang** — Kupplung, Wandler, Getriebe, Fahrzeug | `src/transmission.cpp`, `src/ratio_clutch_constraint.cpp`, `src/torque_converter_constraint.cpp`, `src/vehicle.cpp` | [driveline.md](driveline.md)
**Skriptknoten** — die `.mr`-Seite | `scripting/include/powertrain_nodes.h`, `powertrain_actions.h`, `control_program_nodes.h` | unten
**Skriptbibliothek** | `es/powertrain/*.mr`, `es/objects/objects.mr` | —

---

## Der eine Weg

Vom Skript bis in die Simulation, mit Datei und Zeile. Wer diesen Pfad einmal
gegangen ist, findet alles andere.

```
assets/main.mr
  │  import "engine_sim.mr"  →  es/powertrain/powertrain.mr
  │
  │  set_powertrain(ecu: engine_control_unit(), tcu: transmission_control_unit())
  ▼
scripting/src/language_rules.cpp:213
  │  registerBuiltinType<EngineControlUnitNode>("__engine_sim__engine_control_unit")
  ▼
scripting/include/powertrain_nodes.h
  │  addInput("idle_speed_warm", &m_parameters.idleSpeedWarm)   ← der .mr-Wert landet
  │  generate(ecu)  →  ecu->initialize(parameters)                 im C++-Feld
  ▼
scripting/include/powertrain_actions.h:163
  │  Compiler::output()->powertrain = unit
  ▼
src/engine_sim_application.cpp:648
  │  scriptedPowertrain = output.powertrain
  ▼
src/powertrain_bootstrap.cpp
  │  selectControllers(unit, program)   ← die drei Betriebsarten, siehe unten
  │  system.setController(...) ; system.attach(simulator)
  │  system.registerParameters(registry)   ← ab hier ist alles im Browser sichtbar
  ▼
src/powertrain_system.cpp:461  PowertrainSystem::update(dt)
  │  sampleState        ← Simulation lesen  → PowertrainState
  │  conditionInputs    ← Fahrermodell (Pedalfilter)
  │  controller->update(dt, state, inputs, &commands)
  │  overlay->update(...)          (nur in der Betriebsart ScriptOverlay)
  │  adaptation->update(...)       ← das Lernen
  │  applyCommands     ← ActuatorCommands → Simulation schreiben
  ▼
Drosselklappe, Zündung, Kraftstoff, Gangwahl, Kupplungsdrücke
```

Die vier Substantive, die den Vertrag tragen — sie kommen in jeder Reglerschnittstelle vor:

- **`PowertrainState`** — was die Simulation gerade tut (Drehzahl, Moment, Gang, Temperaturen)
- **`DriverInputs`** — was der Mensch will (Pedal, Bremse, Kupplung, Wählhebel)
- **`ActuatorCommands`** — was der Regler stellt (Klappe, Zündung, Gang, Drücke)
- **`PowertrainBus`** — was die Steuergeräte einander sagen (Momentenrücknahme, Drehzahlwunsch, Schaltung läuft)

`PowertrainController::update(dt, state, inputs, commands)` ist die einzige
Methode, die ein Regler haben muss. ECU, TCU, das Blockprogramm und der
Durchreicher implementieren alle dieselbe.

---

## Die drei Betriebsarten

Was ein Skript einhängt, entscheidet, wer regelt. Die Weiche ist
`powertrain::selectControllers` (`src/powertrain_bootstrap.cpp`):

Skript enthält | Betriebsart | Wer regelt | Wer überlagert
---|---|---|---
nur `set_control_program(...)` | `ScriptOnly` | das Blockprogramm | —
nur `set_powertrain(...)` | `ControlUnits` | ECU + TCU | —
beides | `ScriptOverlay` | ECU + TCU | das Blockprogramm, **nach** ihnen

In `ScriptOverlay` läuft das Blockprogramm im selben Takt hinterher und sieht
die bereits geschriebenen `ActuatorCommands` als Ausgangswert, statt bei null
anzufangen — es korrigiert also, statt zu ersetzen.

**Achtung, ein Wort mit zwei Bedeutungen:** „Overlay" heißt hier die
Regler-Überlagerung. In [calibration_ui.md](calibration_ui.md) heißt „Overlay"
ein Fahrmodus-Parametersatz, der Werte in die Registry schiebt. Zwei
verschiedene Dinge.

**Und eine Falle:** die Adaption hängt sich nur an, wenn ECU und TCU regeln
(`src/powertrain_bootstrap.cpp`, `adaptationAttached`). In `ScriptOnly` gibt es
keinen `AdaptationManager` — ein `learner`-Block lernt trotzdem, weil er direkt
in die Registry schreibt, aber die vier eingebauten Lernpfade laufen nicht.

---

## Die Skriptseite

Zwei Sorten Knoten, und die Regel dahinter ist die Basisklasse — nicht das,
was sie tun:

- **`*_nodes.h`** — leiten von `ObjectReferenceNode<T>` ab. Sie *sind* ein Wert,
  den andere Knoten referenzieren: `engine_control_unit()`, `map_2d()`,
  `pid_controller()`, `thermal()`. Jeder ist ein Träger aus `addInput`-Zeilen,
  die direkt auf die Adresse des echten C++-Felds zeigen.
- **`powertrain_actions.h`** — leiten direkt von `Node` ab, geben nichts aus und
  existieren nur für ihre Wirkung: `set_powertrain(...)`, `add_gear(...)`,
  `set_adaptive(...)`.

Wer einen neuen Knoten braucht, kopiert `ThermalNode`
(`scripting/include/powertrain_nodes.h`) — das ist der kleinste vollständige
Fall: Kanaltyp in `channel_types.h`/`.cpp`, Knotenklasse, Eintrag in
`language_rules.cpp`, `public node` in einer `.mr`, ein Verbraucher.

---

## Tests als Dokumentation

Die Tests sind die genauste Beschreibung des Verhaltens, die es gibt.

Frage | Testdatei
---|---
Was macht die ECU? | `test/ecu_tests.cpp`
Wann und wie schaltet das Getriebe? | `test/gearbox_strategy_tests.cpp`, `test/tcu_tests.cpp`
Was lernt die Adaption, und wann nicht? | `test/adaptation_tests.cpp`
Kommen Skriptwerte wirklich an? | `test/script_tests.cpp`
Kommen Reglerbefehle wirklich in der Simulation an? | `test/powertrain_attach_tests.cpp`
Was kann ein Blockprogramm? | `test/control_program_tests.cpp`, `test/learner_tests.cpp`
Wählhebel und Sperren | `test/selector_gate_tests.cpp`
Registry, Grenzen, Adaptiv-Freigabe | `test/parameter_registry_tests.cpp`, `test/single_source_tests.cpp`
Browser-Schnittstelle | `test/config_server_tests.cpp`

Bauen und laufen lassen steht in [building-on-linux.md](building-on-linux.md).
Vier Fehlschläge sind vorbestehend und nicht Teil dieser Arbeit:
`GasSystemTests.PressureEquilibriumMaxFlow*` und `FunctionTests.FunctionGaussianTest`.
