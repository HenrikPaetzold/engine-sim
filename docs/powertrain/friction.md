# Reibung, Öl und Wärme

Ein Motor verliert Energie an drei Stellen, und alle drei hängen an der
Öltemperatur. Diese Datei beschreibt die Modelle, ihre Parameter und den
Regelkreis, der sich daraus schließt.

**Ab Werk ist nichts davon an.** Ohne `friction(...)` im Motorskript rechnet die
Simulation exakt wie vorher: `vogel_b: 0` heißt Viskositätsfaktor 1, alle
Chen-Flynn-Beiwerte sind 0, und der Ölkühler hat Leitwert 0.

---

## Wo die Reibung entsteht

Ort | Modell | Datei
---|---|---
Kolben gegen Zylinderwand | Stribeck-Kurve | `src/combustion_chamber.cpp`, `frictionForce`
Kurbelwelle und Lager | Chen-Flynn-FMEP | `src/engine_friction.cpp`, `crankTorque`
Öl als Träger von beidem | Vogel-Gleichung | `src/engine_friction.cpp`, `viscosity`

---

## Die Stribeck-Kurve

Reibkraft über Gleitgeschwindigkeit. Sie fällt erst und steigt dann wieder,
weil drei verschiedene physikalische Vorgänge sich ablösen:

```
F
│╲
│ ╲___                          ╱  ← hydrodynamisch: voller Ölfilm
│     ╲___              ______╱
│         ╲____________╱
│  Misch-      Minimum
└──────────────────────────────── v
 Grenzreibung
```

- **Grenzreibung** (v ≈ 0) — Metall auf Metall, nur der Additivfilm trennt.
  Parameter `breakaway_friction` (Losbrechkraft) und `cylinder_friction`
  (Reibbeiwert μ). Deshalb ist Anfahren schwerer als Rollen.
- **Mischreibung** — der Ölfilm baut sich auf, trägt aber noch nicht.
  Die abfallende Flanke, geformt von `breakaway_velocity`.
- **Hydrodynamik** — der Ölfilm trägt vollständig, die Flächen berühren sich
  nicht mehr. Reine Flüssigkeitsscherung, `viscous_friction`.

Die Normalkraft ist nicht geschätzt, sondern abgegriffen:
`Piston::calculateCylinderWallForce()` liefert die Seitenkraft, mit der die
Pleuelschräglage den Kolben gegen die Laufbahn drückt.

```
F = √2·e·(F_brk − F_coul)·exp(−(v/v_st)²)·(v/v_st)   Stribeck-Buckel
  + F_coul·tanh(v/v_coul)                            Coulomb, weich durch null
  + c_visc·v                                         Hydrodynamik
```

`frictionForce(v, wandkraft)` ist absichtlich vom Kolben getrennt: das Reibgesetz
ist ohne Physiksystem prüfbar, und `test/friction_tests.cpp` fährt genau diese
Kurve ab.

---

## Die Vogel-Gleichung

Warum die Öltemperatur überhaupt wirkt. **Newtons Schergesetz**: in einem
Ölfilm der Dicke `h` mit Relativgeschwindigkeit `v` ist die Kraft
`F = μ·A·v/h`. Der Faktor `μ·A/h` ist genau `viscous_friction` — der Beiwert
ist also **proportional zur dynamischen Viskosität**. Das ist keine
Modellannahme, das ist die Definition.

Und die Viskosität hängt brutal an der Temperatur:

```
ν(T) = vogel_k · exp( vogel_b / (T − vogel_theta) )        T in Kelvin
```

Drei Parameter, in der Schmierstofftechnik Standard, von −40 bis 150 °C
brauchbar. Für ein 5W-30 passen `vogel_k: 0.123`, `vogel_b: 948`,
`vogel_theta: 160`:

Öltemperatur | ν [mm²/s] | Faktor gegen 100 °C
---|---|---
−10 °C | ~1210 | 115×
20 °C (Kaltstart) | ~152 | 14×
40 °C | 60 | 5.7×
100 °C (warm) | 10.5 | 1×

**Das ist der Kern.** Bei Kaltstart ist der hydrodynamische Reibungsanteil
vierzehnmal so groß wie warm. Daher braucht ein kalter Motor im Leerlauf mehr
Klappe, daher der hohe Verbrauch auf den ersten Kilometern.

Gerechnet wird nicht mit ν selbst, sondern mit dem **Verhältnis**
`viscosityRatio(T) = ν(T) / ν(reference_temperature)`. Bei
`reference_temperature` ist es exakt 1, und bei `vogel_b: 0` immer 1 — das ist
der Riegel für das Standardverhalten.

### Wenn das Öl zu heiß wird

Bei 200 °C ist ν so klein, dass der Ölfilm zusammenbricht. Die Reibung wird
dann **nicht** kleiner, sondern springt zurück in die Grenzreibung — Metall auf
Metall. `boundary_exponent` bildet das ab:

```
F_brk und F_coul  ×=  viscosityRatio ^ (−boundary_exponent)
```

Bei `boundary_exponent: 0` (Vorgabe) passiert nichts. Bei 0.5 verdoppelt sich
die Grenzreibung, wenn die Viskosität auf ein Viertel fällt. Über die
Öltemperatur aufgetragen ergibt das wieder eine Stribeck-Form: Reibung fällt,
erreicht ein Minimum, steigt wieder.

---

## Chen-Flynn

Ein konstantes Bremsmoment an der Kurbelwelle ist physikalisch falsch — die
Lagerreibung steigt mit Drehzahl und Last. Der Industriestandard (GT-Power,
AVL) rechnet den **FMEP**, den *Friction Mean Effective Pressure*: die
Reibungsarbeit pro Arbeitsspiel, geteilt durch das Hubvolumen, ausgedrückt als
Druck. Man rechnet Reibung als Druck, weil sie sich so direkt mit dem
Mitteldruck der Verbrennung vergleichen lässt.

```
FMEP = constant_fmep                                   Ventiltrieb, Dichtungen
     + peak_pressure_factor · p_max                    Last auf den Lagerschalen
     + speed_factor · v_mp · viscosityRatio            hydrodynamisch
     + speed_squared_factor · v_mp²                    Turbulenz und Planschen
```

`v_mp` ist die mittlere Kolbengeschwindigkeit, `2 · Hub · Drehzahl[1/s]`.
**Nur der `speed_factor`-Term hängt an der Viskosität** — er ist der
hydrodynamische; der Turbulenzterm hängt an der Dichte, nicht an der Zähigkeit.

Umrechnung in ein Moment, exakt und ohne Näherung:

```
T = FMEP · Hubraum / (4π)        Viertakt: ein Spiel = 2 Umdrehungen = 4π rad
```

Beispiel 2 L bei 3000/min mit FMEP 1 bar: `T ≈ 15.9 N·m`, Reibleistung
`T·ω ≈ 5 kW`.

`p_max` kommt aus einem Spitzenwerthalter über die Zylinderdrücke, der mit
`peak_pressure_decay` abklingt — sonst müsste man einen ganzen Arbeitszyklus
puffern.

**Mit allen vier Beiwerten auf 0 ist das Moment exakt 0**, und es bleibt allein
`crankshaft.friction_torque` — also genau das Verhalten vor dieser Änderung.

---

## Der geschlossene Kreis

Reibleistung ist `F·v` am Kolben und `T·ω` an der Kurbelwelle. Über den
Zeitschritt integriert ergibt das Energie — und die geht **ins Öl**, denn Lager
und Kolbenhemd werden vom Öl gekühlt, nicht vom Kühlwasser.

```
kaltes Öl → hohe Viskosität → hohe Reibung → viel Reibungswärme
     ↑                                              │
     └──────────────  Öl wird warm  ←───────────────┘
```

Eine negative Rückkopplung, die sich anfangs selbst beschleunigt — deshalb wird
ein Motor unter Last viel schneller warm als im Leerlauf.

`heat_to_oil` teilt die Reibungswärme auf: 1.0 (Vorgabe) alles ins Öl, 0.0 alles
in den Block. Der Weg ist `ThermalModel::addOilHeat`, ein zweiter Eingang neben
`addHeat` — davor konnte Öl nur über den Block warm werden.

Das ist zugleich eine geschlossene Energiebilanzlücke: die Reibkraft wurde der
Mechanik schon immer entzogen, die Energie ist bisher nur verschwunden.

---

## Kühlung, und wie heiß es werden kann

Das Öl hat drei Abflüsse:

1. `thermal.oil_to_ambient` — Ölwanne an Umgebungsluft, fest, immer aktiv.
2. **Der Block als Kühlkörper**, sobald das Öl heißer ist als er:
   `blockToOil · (T_block − T_öl)` dreht dann das Vorzeichen. Der Block hängt am
   Wasserkühler mit sehr hoher Autorität und sitzt praktisch auf
   Thermostattemperatur.
3. `thermal.oil_cooler` — der **Ölkühler**, mit eigenem Thermostat
   (`oil_thermostat_open`/`_full`, Vorgabe 90…105 °C) und luftstromabhängig wie
   der Wasserkühler. **Vorgabe 0 = kein Ölkühler.**

Ohne Ölkühler, mit Block auf 85 °C und Umgebung 20 °C, gilt näherungsweise:

```
T_Öl ≈ (Reibleistung[W] + 5700) / 90        [°C]
```

Reibleistung | Öltemperatur
---|---
3 kW (Landstraße) | 97 °C
6 kW (Vollgas) | 130 °C
10 kW | 193 °C — das Öl ist tot

Es deckelt sich also selbst, aber nur schwach: man kann den Motor sehr wohl zum
Kochen bringen. Wer das sehen will, dreht `friction.speed_factor` hoch und lässt
`thermal.oil_cooler` auf 0. Wer es verhindern will, schaltet den Ölkühler zu und
sieht die Beharrungstemperatur wandern.

---

## Was im Browser sichtbar ist

Alle Parameter liegen unter `friction.*` und `thermal.oil_*` und sind live
verstellbar. Dazu zwei Telemetriewerte und zwei Oszilloskopkanäle:

Anzeige | Kanal | Bedeutung
---|---|---
`viscosity` | `oil_viscosity` | ν in mm²/s, aus der Vogel-Gleichung
`friction` | `friction_power` | Reibleistung in kW, Kolben plus Kurbelwelle

Damit lässt sich der Kreis von oben direkt beobachten: Öltemperatur steigt,
Viskosität fällt, Reibleistung fällt nach — bis der `boundary_exponent` sie
wieder hochzieht.

---

## Die Parameter auf einen Blick

`friction(...)` im Motorskript, alle auch als Schieber:

Eingang | Vorgabe | Wirkung
---|---|---
`constant_fmep` | 0 | Chen-Flynn A, drehzahlunabhängig
`peak_pressure_factor` | 0 | Chen-Flynn B, Lastterm
`speed_factor` | 0 | Chen-Flynn C, hydrodynamisch, viskositätsabhängig
`speed_squared_factor` | 0 | Chen-Flynn D, Turbulenz
`vogel_k` | 0.123 | Vogel-Vorfaktor, mm²/s
`vogel_b` | **0** | Vogel-Steigung; 0 schaltet die Temperaturabhängigkeit ab
`vogel_theta` | 160 | Vogel-Polstelle, K
`reference_temperature` | 100 °C | wo das Viskositätsverhältnis 1 ist
`peak_pressure_decay` | 0.5 s | Abklingzeit des Spitzendruckhalters
`heat_to_oil` | 1.0 | Anteil der Reibungswärme ins Öl statt in den Block
`cylinder_friction` | 0.06 | Coulomb-Reibbeiwert μ am Kolben
`breakaway_friction` | 50 N | Losbrechkraft
`breakaway_velocity` | 0.1 m/s | Breite des Stribeck-Buckels; 0 schaltet ihn ab
`viscous_friction` | 20 N·s/m | hydrodynamischer Beiwert am Kolben
`boundary_exponent` | **0** | wie stark dünnes Öl die Grenzreibung anhebt

Am Thermomodell dazu: `oil_cooler` (Vorgabe **0**), `oil_thermostat_open`
(90 °C), `oil_thermostat_full` (105 °C).
