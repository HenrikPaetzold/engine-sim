# Antriebsstrang: übersetzungstragende Constraints

## Das Problem des alten Modells

`Transmission::changeGear()` hat die Übersetzung in die Trägheit des
Antriebsstrangkörpers gerechnet:

```
f      = r_Reifen / (i_Achse · i_Gang)
I_neu  = m_Fahrzeug · f²
```

und `v_theta` energieerhaltend nachgezogen. Daraus folgen drei Beschränkungen:

1. Ein Gangwechsel ist ein Sprung — er hat keine Dauer und lässt sich nicht modulieren.
2. Es kann immer nur **eine** Übersetzung gleichzeitig wirken. Ein DCT braucht zwei.
3. Zwischen Kurbelwelle und Antriebsstrang sitzt genau ein Körper. Für einen
   Wandler fehlt die Turbine.

## Das neue Modell

Der Antriebsstrangkörper bekommt eine **konstante** Trägheit, die Übersetzung
wandert in den Constraint.

```
f₀    = r_Reifen / i_Achse
I_AS  = m_Fahrzeug · f₀²
```

`f₀` bezieht den Körper auf die **Getriebeausgangswelle**: `ω_AS` ist die
Kardanwellendrehzahl, `v = f₀ · ω_AS`.

### RatioClutchConstraint

Geschwindigkeitsbedingung zwischen Eingang (Kurbelwelle oder Turbine) und Ausgang:

```
−ω_ein + i · ω_aus = 0
J = [0, 0, −1 | 0, 0, i]
limits = ± (Kapazität · Druck)
```

Der Solver liefert λ als Constraint-Moment. Damit ist

```
M_ein = −λ          M_aus = +i · λ
```

also **λ das eingangsseitige Moment** — dieselbe Bedeutung, die
`MaxClutchTorque` schon vorher hatte. Die Leistungsbilanz

```
P = −λ ω_ein + i λ ω_aus = λ (i ω_aus − ω_ein) = 0
```

verschwindet, solange die Bedingung erfüllt ist; im Schlupf (Limit aktiv) wird
genau die Schlupfleistung dissipiert.

### Äquivalenz zum alten Modell

Aus Sicht des Motors ist die wirksame Trägheit

```
I_eff = I_Kurbel + I_AS / i²
      = I_Kurbel + m · f₀² / i²
      = I_Kurbel + m · (r / (i_Achse · i))²
```

— identisch mit `I_neu` des alten Modells. Ebenso bleibt

```
v = f₀ · ω_AS = (r / i_Achse) · (ω_Motor / i) = ω_Motor · r / (i_Achse · i)
```

unverändert, sodass `Vehicle::getSpeed()` und
`Vehicle::linearForceToVirtualTorque()` (beide über `sqrt(I/m) = f₀`)
ohne Änderung weiter gelten. Verifiziert in
`GearboxModelTests.TheConstantDrivelineInertiaReproducesTheLegacyGearInertia`
und `RatioClutchTests.TheEffectiveInertiaFollowsTheSquareOfTheRatio`.

### Doppelkupplung

Zwei `RatioClutchConstraint` parallel auf denselben Antriebsstrangkörper, mit
den Übersetzungen des aktuellen und des vorgewählten Gangs. Beim Überblenden
sind beide teilweise geschlossen; der LCP-Solver verteilt das Moment auf beide
Limits. Zugkraftunterbrechung entfällt.

### Wandler

Zusätzlicher Turbinenkörper zwischen Kurbelwelle und Getriebe.

```
SR = ω_Turbine / ω_Pumpe            (SAE-Drehzahlverhältnis)
TR(SR)                              Momentenverhältnis
M_P = C(SR) · ω²                    Pumpenmoment
```

Als Constraint:

```
J = [0, 0, −1 | 0, 0, TR(SR)]
limits = ± C(SR) · ω_ref²
```

Das Limit ist im Betrieb praktisch immer aktiv — der Constraint wirkt dann als
reine Momentenquelle mit `M_P` an der Pumpe und `TR · M_P` an der Turbine, also
genau als Wandler. Erst wenn `SR` gegen den Kupplungspunkt läuft, wird `TR = 1`
und der Constraint geht in eine starre Kopplung über.

**Standardkurven** (überschreibbar über `Function`-Kurven):

```
TR(SR) = TR_stall + (1 − TR_stall) · SR / SR_c      für SR < SR_c, sonst 1
C(SR)  = C₀ · (1 − SR²)
```

`SR_c = 0.85`, `TR_stall = 2.0`. Der Kapazitätsfaktor `C₀ = 4.04e-3 Nm/(rad/s)²`
entspricht einem K-Faktor von ca. 175 rpm/√(lb·ft):

```
M_P [lb·ft] = (n_P [rpm] / K)²
n_P = 2000 rpm, K = 175  →  M_P = 130 lb·ft = 177 Nm
ω_P = 209.4 rad/s        →  C₀ = 177 / 209.4² = 4.04e-3
```

Die Stall-Drehzahl folgt daraus als `ω = sqrt(M_Motor / C₀)` — verifiziert in
`TorqueConverterTests.TheStallSpeedFollowsTheCapacityFactor`.

`ω_ref` ist der Betrag der größeren der beiden Drehzahlen. Damit bleibt der
Wandler auch im Schub wirksam, wenn die Turbine schneller läuft als die Pumpe;
`TR` ist dort auf 1 begrenzt, weil ein Wandler rückwärts nicht multipliziert.

## Schaltstrategie

Die Constraints tragen die drei Bauarten; welche Kupplung wann welchen Druck
bekommt, entscheidet die TCU.

### Doppelkupplung: Parität

`clutchForGear(g) = g % 2`. Gang 1 und 3 liegen auf Kupplung A, Gang 2 und 4 auf
B. Die TCU führt `m_clutchGear[2]` und `m_activeClutch` und schickt die Zuordnung
über `ActuatorCommands::clutchGear[]` an `Transmission::setClutchGear()`; dort
liest `updateRatioClutches()` sie statt der früheren Konvention
„Kupplung 0 = aktueller Gang, Kupplung 1 = vorgewählter Gang". Setzt niemand die
Zuordnung, fällt das Getriebe auf diese Konvention zurück.

Im Ruhezustand legt `preselectedNeighbour()` auf der freien Kupplung den Gang an,
dem der Schaltplan am nächsten ist: der Abstand zur Hochschaltschwelle wird gegen
den Abstand zur Rückschaltschwelle gehalten, beide aus `m_upshiftMap` und
`m_downshiftMap`. Bei Vollgas steht der nächsthöhere, im Schub der
nächstniedrigere Gang bereit.

Ein Sprung über zwei Gänge trifft dieselbe Parität und damit dieselbe Welle.
`beginShift()` erkennt das (`sameShaft`) und fällt auf `TorqueReduction` zurück —
öffnen, umlegen, schließen. Das ist kein Notnagel, sondern was die Hardware tut.

### Überblendung mit Rückführung

`ClutchOverlap` ist eine Zeitrampe als Vorsteuerung plus die ILC-Korrektur:

```
oncoming = t + engageProfile.correction(t)
offgoing = (1 − t) + overlapHold · (1 − |2t − 1|)
```

`overlapHold` hält beide Kupplungen in der Mitte der Überblendung zusätzlich
geschlossen, damit das Moment nie abreißt. `m_engagePhase` wird auch hier
gesetzt, und das Tor der Adaption
(`src/adaptation/adaptation_manager.cpp`) steht auf
`ClutchEngage || ClutchOverlap` — damit lernt das DCT seine Schaltungen wie das
AMT.

### Die Form einer Schaltung

Die Rampe in `ClutchOverlap` und `ClutchEngage` war fest verdrahtet: der Druck
der kommenden Kupplung stieg linear mit der Phase. Zeiten und Schwellen ließen
sich verstellen, die **Form** nicht — und damit ließ sich derselbe
Antriebsstrang nicht einmal knackig und einmal weich schalten.

Zwei Kennfelder tragen sie jetzt:

```
tcu.overlap_shape    x = Phase 0…1, y = Pedal → Druck der kommenden Kupplung
tcu.engage_shape     dasselbe für das Einkuppeln
```

Die Vorgabe ist die Identität, also exakt die bisherige lineare Rampe; ein
Skript ohne die Eingänge `overlap_shape` / `engage_shape` verhält sich
unverändert. Die ILC-Korrektur bleibt unverändert obendrauf, die abgehende
Kupplung folgt als Gegenstück (`1 − Form` plus `overlapHold`).

Weil ein Fahrmodus Kennfelder tragen kann (siehe `calibration_ui.md`), ist der
Schaltcharakter damit reine Bedatung — ein kurzes steiles Profil gegen ein
langes weiches, auf derselben Hardware:

```
add_drive_mode(drive_mode(name: "sport_plus")
    .set("tcu.shift.clutch_overlap_time", 0.08)
    .set_map(path: "tcu.overlap_shape", map: map_2d()
        .add_map_sample(x: 0.0,  y: 0.0, value: 0.0)
        .add_map_sample(x: 0.25, y: 0.0, value: 1.0)
        .add_map_sample(x: 1.0,  y: 0.0, value: 1.0)))
```

### Kickdown: Ziel statt fester Drehzahl

Der Kickdown suchte den niedrigsten Gang, dessen Drehzahl unter **fest
verdrahteten 6200 rpm** blieb. Die Zahl hing weder am Drehzahlbegrenzer noch an
einem Skripteingang: ein Diesel mit 4500 rpm Abregelung griff damit in einen
Gang, den er nicht drehen kann, ein Motorradmotor nutzte seinen Bereich nie.

Jetzt gibt `tcu.kickdown_map` (x = Pedal) die **Solldrehzahl nach der
Rückschaltung** vor, und die TCU wählt den niedrigsten Gang, der sie nicht
überschreitet. Das ist übersetzungsunabhängig und damit für jede Motorbauart
richtig. Zusätzlich wird hart gegen den Drehzahlbegrenzer der ECU begrenzt
(`kickdown_rev_margin` als Abstand darunter) — eine Fehlbedatung kann den Motor
nicht überdrehen.

Als zweiter Auslöser dient ein gefilterter **Pedalgradient**: steigt das Pedal
schneller als `kickdown_pedal_rate` und liegt über `kickdown_pedal_floor`, löst
derselbe Mehrfachsprung aus, ohne dass die Absolutschwelle erreicht sein muss.
Ein energischer kurzer Gasstoß schaltet also zurück, langsames Durchtreten nicht.
Die Vorgabe ist 0, der Gradient also aus — bestehende Skripte verhalten sich
unverändert.

Beides ist registriert und damit **pro Fahrmodus** verschiebbar:

```
add_drive_mode(drive_mode(name: "sport_plus")
    .set("tcu.kickdown.pedal_rate", 2.0)
    .set_map(path: "tcu.kickdown_map", map: map_2d()
        .add_map_sample(x: 0.0, value: 6000 * units.rpm)
        .add_map_sample(x: 1.0, value: 7000 * units.rpm)))

add_drive_mode(drive_mode(name: "comfort")
    .set("tcu.kickdown.pedal_rate", 50.0))
```

Mit der Standardfilterung von 80 ms liegt die höchstmögliche beobachtbare Rate
bei etwa 12,5 1/s; ein Wert darüber schaltet den Gasstoß-Auslöser also ab.

### Wandler: Kennfeld und Schlupfregler

`m_lockupMap` hat dieselbe Form wie die Schaltkennfelder (x = Pedal, y = Gang),
der Wert ist die Geschwindigkeit, ab der überbrückt wird. Drei Zustände:

| | Bedingung |
|---|---|
| offen | unter der Kennfeldschwelle, während einer Schaltung, oder Pedal über `kickdownThreshold` |
| schlupfend | über der Schwelle — der Regler führt `converterSlip` auf `lockupSlipTarget` |
| zu | Schlupf unter `lockupLockSlip` |

Der Regler ist rückwärtswirkend (`update(dt, −Sollschlupf, −Istschlupf)`): mehr
Schlupf heißt mehr Druck. Sein Ausgang läuft durch einen Ratenbegrenzer
(`lockupApplyRate`, Standard 1.5 1/s), der die hydraulische Füllzeit abbildet —
zugehen dauert, aufgehen ist sofort. Dieselbe Struktur regelt in
`launchPressure()` die Anfahrkupplung.

Kriechen fällt aus der Physik: in einer Vorwärtsraste liefert `launchPressure()`
für `hasLaunchDevice` konstant 1.0, der Gang ist eingelegt, und der Wandler
überträgt im Stillstand sein Stall-Moment
(`TorqueConverterTests.TheConverterCreepsForwardWithoutThrottle`).

### AMT: die Zugkraftunterbrechung

`shiftTorqueCut` (Standard 1.0) gilt für `ClutchRelease` und `GearChange`, wo das
Moment vollständig weg soll, damit der Motor auf die neue Synchrondrehzahl
fällt. `shiftTorqueReduction` bleibt für `ClutchOverlap` und `ClutchEngage`, wo
ein Restmoment gewollt ist. Das Zwischengas bei Rückschaltungen läuft über
`m_bus.speedRequest`; die ECU hebt daraufhin den Leerlaufsollwert an und hat dort
volle Stellautorität.

### Vorgaben je Bauart

`es/powertrain/gearboxes.mr` liefert zu jedem Getriebe eine passende
Kalibrierung: `manual_control()`, `robotised_manual_control()`,
`dual_clutch_control()` und `converter_control()`. Sie setzen nur Vorgaben auf
`transmission_control_unit()` — jeder Eingang bleibt einzeln überschreibbar.

## Rückwärtskompatibilität

`Transmission::Type::Legacy` ist der Default und behält das alte Verhalten
Zeile für Zeile. Bestehende Motorskripte ändern sich nicht. Erst
`transmission(type: "manual" | "dct" | "converter")` schaltet auf das neue
Modell um.

Die Fähigkeiten des Getriebes sind die Quelle der Wahrheit für die TCU:
`PowertrainSystem::syncGearbox()` überträgt `supportsPreselect`,
`requiresTorqueInterrupt`, `hasLaunchDevice`, die Übersetzungen, den
Achsantrieb und den Reifenradius in die TCU-Parameter, bevor die Registry
aufgebaut wird. Ein Skript kann also keine Vorwahl auf einem Getriebe
anfordern, das keine zweite Kupplung hat.
