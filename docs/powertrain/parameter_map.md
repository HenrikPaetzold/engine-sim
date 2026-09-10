# Die drei Namen einer Stellgröße

Jede Stellgröße heißt dreimal:

- **`.mr`-Eingang** — das Einzige, was ein Skriptautor tippt
- **C++-Feld** — das Einzige, was im Debugger steht
- **Registry-Pfad** — das Einzige, was der Browser zeigt

Meistens stimmen sie mechanisch überein: der `.mr`-Name ist das Blatt des
Registry-Pfads, der Gruppenpräfix wird vorn abgetrennt.

```
launch_speed   →   launchSpeed   →   tcu.launch.speed
```

Wer das einmal gesehen hat, findet die anderen zwei Namen ohne Suche. Diese
Datei listet **die Stellen, wo das nicht gilt** — dort kostet jede Suche sonst
zwei greps.

---

## Wo die Regel bricht

`.mr`-Eingang | C++-Feld | Registry-Pfad | was fehlt wo
---|---|---|---
`throttle_rate` | `throttleLearningRate` | `adaptation.throttle_map.rate` | „learning" nur in C++, „map" nur im Pfad
`throttle_deadband` | `throttleDeadband` | `adaptation.throttle_map.deadband` | „map" nur im Pfad
`throttle_learn_from_integrator` | `throttleLearnFromIntegrator` | `adaptation.throttle_map.learn_from_integrator` | „map" nur im Pfad
`idle_limit` | `idleTrimLimit` | `adaptation.idle.limit` | „trim" nur in C++
`lambda_gain` | `lambdaShortTermGain` | `adaptation.lambda.short_term_gain` | „short term" fehlt im `.mr`-Namen
`torque_model_min` | `estimateMin` | `adaptation.torque_model.estimate_min` | „estimate" fehlt im `.mr`-Namen
`torque_model_max` | `estimateMax` | `adaptation.torque_model.estimate_max` | dito
`torque_model_covariance` | `initialCovariance` | *(nicht registriert)* | „initial" nur in C++, kein Schieber
`soft_limit_band` | `softLimitBand` | `ecu.limiter.soft_band` | „limit" fällt im Pfad weg, weil die Gruppe schon `limiter` heißt
`hard_limit_offset` | `hardLimitOffset` | `ecu.limiter.hard_offset` | dito
`stall_protect_speed` | `stallProtectSpeed` | `tcu.launch.stall_protect_speed` | der Pfad erfindet die Gruppe `launch`, im `.mr` steht der Eingang flach
`driver_clutch` | `driverClutchAuthority` | `tcu.gearbox.driver_clutch_authority` | `.mr`-Name ist abgeschnitten
`ambient` | `ambientTemperature` | `thermal.ambient_temperature` | „temperature" fehlt im `.mr`-Namen — und das ist **innerhalb des Thermoknotens inkonsistent**, denn `initial_block_temperature` und `initial_oil_temperature` schreiben es aus
`cross_sectional_area` | `crossSectionArea` | `vehicle.cross_section_area` | drei Schreibweisen desselben Worts

---

## Zwei Namen für dieselbe Größe

Diese beiden existieren **doppelt**, mit eigenem Feld und eigenem Schieber:

Größe | am Fahrzeug | am Getriebe
---|---|---
Reifenhalbmesser | `vehicle.tire_radius` | `tcu.gearbox.tire_radius`
Achsübersetzung | `vehicle.diff_ratio` | `tcu.gearbox.final_drive`

Das ist Absicht: die TCU darf mit einer anderen Annahme rechnen als das
Fahrzeug fährt — so kann man sehen, was eine falsch bedatete Steuergeräteseite
anrichtet. Der Regelfall ist aber, dass das Skript die TCU-Werte **nicht**
setzt; dann übernimmt sie die des Fahrzeugs (`configureGearbox` in
`src/powertrain/transmission_control_unit_defaults.cpp`, geschützt durch die
Authored-Flaggen).

**Wer am Fahrzeugschieber dreht und nichts merkt, hat den falschen von beiden
erwischt.**

---

## Nur im Browser, nicht im Skript

Registriert und live einstellbar, aber ohne `.mr`-Eingang — sie überleben also
keinen Neustart:

- `vehicle.road_grade`
- sämtliche `program.*` eines Blockprogramms (die kommen aus den Blöcken selbst)

Umgekehrt gibt es keinen Fall: alles, was ein `.mr`-Eingang setzt, ist auch
registriert.

---

## Wo die Gruppen herkommen

Der Registry-Pfad wird in `registerParameters` zusammengesetzt, jeweils in
`*_parameters.cpp`:

Präfix | gesetzt in
---|---
`ecu.` | `src/powertrain/engine_control_unit_parameters.cpp`
`tcu.` | `src/powertrain/transmission_control_unit_parameters.cpp`
`adaptation.` | `src/adaptation/adaptation_manager.cpp`
`thermal.` | `src/thermal_model.cpp`
`vehicle.` | `src/vehicle.cpp`
`program.` | `src/powertrain/scripted_control_unit.cpp`
`starter.`, `driver.`, `control.` | `src/powertrain_system.cpp`

Die Oberfläche schneidet den Pfad selbst auf: **die ersten zwei Segmente werden
die Gruppenüberschrift, der Rest die Beschriftung** (`groupOf` in
`assets/config_ui/index.html`). `tcu.launch.speed` landet also unter „tcu.launch"
mit der Beschriftung „speed". Das weiß nur das JavaScript — im C++ steht davon
nichts.
