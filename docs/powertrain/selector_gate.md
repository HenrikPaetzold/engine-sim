# Wählhebel: freie Rasten statt festem PRND

## Warum keine Enum

Der erste Entwurf hatte `enum class DriveRange { Park, Reverse, Neutral, Drive }`
fest im Code — derselbe Fehler wie ein festes Sport/Comfort/Eco bei den Fahrmodi.
Ein Schubhebel REV–IDLE–CLB–MCT–TOGA ist dieselbe Maschine wie P–R–N–D, nur mit
anderen Rasten. Wer die Namen in den Code schreibt, hat den zweiten Fall
ausgeschlossen.

## Die Trennlinie: Physik geschlossen, Semantik offen

Ein Antriebsstrang kann **physikalisch** nur vier Dinge:

```cpp
enum class GateEngagement { Park, Reverse, Neutral, Forward };
```

Das ist geschlossen, weil es nicht mehr gibt: gesperrt, rückwärts übersetzt,
getrennt, vorwärts übersetzt. Alles andere ist Semantik und gehört ins Skript:

```cpp
struct GatePosition {
    std::string name;                  // "D", "TOGA", "Crawl", was du willst
    GateEngagement engagement;         // was die Physik tut
    double maxEntrySpeed = -1.0;       // Verriegelung: rein
    double maxExitSpeed = -1.0;        // Verriegelung: raus
    bool requiresBrake = false;        // Bremsverriegelung
    std::string mode;                  // Parameter-Overlay, das die Raste zieht
};
```

`SelectorGate` ist die geordnete Liste. Der Hebel geht **eine Raste pro Regeltakt**,
nie im Sprung — wie ein echter Hebel durch die Gasse.

## Was eine Raste komponiert

Genau das, wonach du gefragt hast: eine Raste wählt eine bestehende Bedatung aus
*und* verstellt bei Bedarf ECU und TCU.

```
GatePosition
   ├── engagement  ──► Transmission: Sperrklinke / −i_R / Kupplung auf / i_Gang
   ├── mode        ──► DriveModeSet::select() ──► Registry ──► ECU + TCU Parameter
   └── Verriegelung ─► TCU lehnt den Rastenwechsel ab, wenn die Bedingung nicht hält
```

Der `mode`-Weg ist derselbe, den auch `L` im Spiel und die Knöpfe im Browser
benutzen — eine Raste ist damit ein Fahrmodus mit angehängter Physik.
`PowertrainSystem::applyGateMode()` zieht das Overlay, sobald sich die Raste ändert.

## Beispiel: Automatik

```
transmission_control_unit(default_position: "P")
    .automatic_gate()
```

`automatic_gate()` ist nur eine Bibliotheksfunktion, die vier `gate_position`
aufreiht. Sie steht in `es/powertrain/powertrain.mr` und hat keinen Sonderstatus.

## Beispiel: Schubhebel

```
transmission_control_unit(default_position: "IDLE")
    .add_gate_position(
        gate_position(name: "REV", engagement: "reverse",
                      max_entry_speed: 1.0, mode: "reverse_thrust"))
    .add_gate_position(gate_position(name: "IDLE", engagement: "neutral"))
    .add_gate_position(gate_position(name: "CLB",  engagement: "forward", mode: "climb"))
    .add_gate_position(gate_position(name: "MCT",  engagement: "forward", mode: "mct"))
    .add_gate_position(gate_position(name: "TOGA", engagement: "forward", mode: "toga"))
```

Dazu die passenden Overlays, die dann ECU und TCU verstellen:

```
add_drive_mode(drive_mode(name: "toga")
    .set("ecu.limiter.rev_limit", 8800 * units.rpm)
    .set("ecu.torque_rise_rate", 6000 * units.Nm)
    .set("tcu.shift.min_gear_time", 0.25))
```

Verifiziert in `SelectorGateTests.AFreelyNamedGateBehavesLikeAThrustLever`: REV
wird bei 40 m/s abgelehnt und bei 0,4 m/s angenommen, TOGA und CLB melden ihr
Overlay.

## Rückwärtsgang

`RatioClutchConstraint` trägt die Übersetzung im Jacobian
(`src/ratio_clutch_constraint.cpp:37`), also ist Rückwärts einfach eine **negative
Übersetzung**: `−i_R` erzwingt `ω_ein = −i_R · ω_aus`, die Leistungsbilanz bleibt
null, und die Momentengrenzen gelten unverändert. Kein neuer Constraint.

Der Legacy-Pfad kann das strukturell nicht: er rechnet `I = m·(r/(i_Achse·i))²`,
das Vorzeichen wird wegquadriert. `Transmission::supportsEngagement()` meldet das,
die TCU verweigert dort Park und Reverse und bleibt in Neutral.

## Parksperre

`atg_scs::RotationFrictionConstraint` — existierte bereits im Solver: ein Körper,
`J_θ = +1`, Ziel `v_θ = 0`, **symmetrische** Momentengrenzen. Das ist exakt eine
Sperrklinke mit endlichem Haltemoment. Auf `±ParkLockTorque` bei Park, sonst `0/0`.

Am Berg hält sie, bis das Hangabtriebsmoment `ParkLockTorque` übersteigt — dann
rutscht sie durch, wie eine echte Klinke. Verifiziert gegen den Solver in
`ParkLockTests.TheParkLockHoldsAgainstAGradeAndSlipsBeyondItsTorque`.

## Zwei Fehler, die dabei herauskamen

**Die Vorzeichenkonvention war invertiert.** Vorwärts ist intern *negatives*
`v_theta` (`Engine::isSpinningCw`, `StarterMotor`, `IgnitionModule`). `getSignedSpeed()`
lieferte deshalb beim Vorwärtsfahren einen negativen Wert — ein Fehler aus M3, den
nur die `std::abs()`-Aufrufe der TCU verdeckt haben. Jetzt richtig herum.

**Der Fahrwiderstand verschwand rückwärts.** `VehicleDragConstraint` verzweigte über
das Vorzeichen der Widerstands*kraft*, nie über die *Fahrtrichtung*; bei `v_theta > 0`
klemmte λ auf 0 und Roll- wie Luftwiderstand waren exakt null. Jetzt getrennt in

```
dissipativ (Roll + Luft + Bremse)  → Richtung immer der Bewegung entgegen
Steigung                            → vorzeichenbehaftet, richtungsunabhängig
Stillstand                          → beidseitiges Fenster (Haftreibung)
```

Der Stillstandsfall ist neu: bei `|Steigungskraft| < Rollwiderstand` bleibt das Auto
stehen, sonst rollt es an. Ohne ihn wäre die Parksperre am Berg nicht prüfbar.

## Bedienung

`O` eine Raste Richtung erste Position, `P` eine Richtung letzte, `K` bremsen.
Die Bremse ist echte Physik (`vehicle.max_brake_force`), nicht nur Verriegelung.

## Voreinstellung und Rückwärtskompatibilität

Ohne `default_position` wählt die TCU die **erste Vorwärtsraste**. Ein Skript ohne
Gassen-Definition bekommt automatisch P–R–N–D und fährt in D los — genau wie vorher.
Wer es realistisch will, setzt `default_position: "P"`.
