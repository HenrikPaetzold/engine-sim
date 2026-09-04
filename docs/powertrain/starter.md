# Anlasser: Kennfeld statt zweier Konstanten

## Vorher

`StarterMotor` (`src/starter_motor.cpp`) ist ein Geschwindigkeits-Constraint auf
der Kurbelwelle: `J_θ = 1`, Ziel `v_bias = −m_rotationSpeed`, Momentengrenzen
`±m_maxTorque`, dazu ein `m_enabled`-Schalter. Beide Werte waren Konstanten aus
`engine(starter_speed:, starter_torque:)`.

Damit war ein Kaltstart von einem Start-Stopp-Neustart nicht zu unterscheiden —
außer durch eine Fallunterscheidung im Code, also genau das, was hier nicht
passieren soll.

## Jetzt

Zwei **optionale** Kennfelder tragen die Charakteristik:

```
starter_torque_map    x = Kurbelwellendrehzahl (Betrag), y = Öltemperatur
                      → verfügbares Moment
starter_speed_map     x = Öltemperatur → Anwerf-Zieldrehzahl
```

Ein Gleichstromanlasser liefert im Stillstand sein größtes Moment, das bis zur
Leerlaufdrehzahl auf null fällt; ein kalter Akku hat einen höheren
Innenwiderstand und liefert weniger. Beides trägt dasselbe Kennfeld, ohne dass
dafür ein elektrisches Modell gebaut werden müsste.

**Die Temperatur wird nicht geschätzt.** Sie kommt pro Schritt aus dem
vorhandenen `ThermalModel` des Motors (`Engine::getOilTemperature()`), gesetzt in
`PistonEngineSimulator::simulateStep_()`.

## Vorzeichen

Vorwärtslauf ist intern **negatives** `v_theta`, und `m_rotationSpeed` ist
entsprechend negativ vorbelegt. Das Kennfeld wird deshalb mit dem *Betrag* der
Drehzahl abgetastet, und `targetSpeed()` übernimmt das Vorzeichen der
Voreinstellung. Verifiziert in
`StarterMotorTests.TheTargetSpeedKeepsTheForwardSign` und
`.TheCrankSpinsForward`.

## Rückwärtskompatibilität

Ohne Kennfeld gelten `starter_torque` und `starter_speed` unverändert —
nachgewiesen in `StarterMotorTests.WithoutAMapTheConstantsStillApply`. Alle
bestehenden Motorskripte laufen damit wie zuvor.

## Vorgaben

```
engine(
    starter_torque_map: dc_starter_torque_map(
        stall_torque: 200 * units.lb_ft,
        free_speed: 300 * units.rpm,
        cold_fraction: 0.55),
    starter_speed_map: dc_starter_speed_map(
        cold_speed: 150 * units.rpm,
        warm_speed: 260 * units.rpm))
```

`dc_starter_torque_map()` und `dc_starter_speed_map()` stehen in
`es/objects/objects.mr` und sind gewöhnliche Bibliotheksknoten ohne Sonderstatus
— dasselbe Muster wie `dual_clutch_control()` bei den Getrieben. Jeder Eingang
bleibt einzeln überschreibbar, und wer ein eigenes Kennfeld bedaten will, gibt
einfach `map_2d()` mit eigenen Stützstellen an.

## Grenze

Die Motorreibung ist temperaturunabhängig: `breakawayFriction` und
`viscousFrictionCoefficient` (`include/combustion_chamber.h`) sind Konstanten.
Kaltes Öl bremst also nicht stärker, obwohl das Thermomodell die Öltemperatur
kennt. Der Kaltstart-Charakter kommt derzeit vollständig aus dem
Anlasserkennfeld. Das zu ändern wäre ein Eingriff in die Motorsimulation selbst
und würde das Warmlaufverhalten aller bestehenden Skripte verschieben — eine
eigene Aufgabe.
