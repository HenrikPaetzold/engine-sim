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
