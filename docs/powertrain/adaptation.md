# Wie das Steuergerät lernt

## Zwei Schleifen, die man nicht verwechseln darf

**Bedatung** passiert vorher und außerhalb: Prüfstand, Versuchsplanung,
Applikationsingenieure. Ergebnis sind Kennfelder *und* Reglerbeiwerte. In diesem
Projekt ist das die `.mr`-Datei und das Webinterface.

**Adaption** passiert im Betrieb: eng begrenzt, bedingungsbewacht, nur gegen
Toleranz, Verschleiß, Kraftstoffqualität und Höhe. Das ist der
`AdaptationManager`.

Was **nicht** adaptiert wird, weder hier noch in einem Seriensteuergerät, sind
die Reglerbeiwerte. `kp`, `ki`, `kd` sind bedatbar und im Browser verstellbar,
aber kein Lernpfad fasst sie an. Gelernt wird die **Vorsteuerung**, damit der
Regler nichts mehr zu tun hat.

## Das Grundmuster: Integratorwanderung

Hält ein I-Anteil an einem Betriebspunkt dauerhaft dieselbe Korrektur, ist das
keine Störung, sondern Wissen. Es wird aus dem Integrator geschöpft und in ein
Kennfeld geschrieben:

```
transfer = rate * integrator * dt
kennfeld.accumulate(x, y, transfer, min, max)
integrator -= transfer
```

Was rausfließt, fließt rein: die **Summe über alle Kennfeldzellen** bleibt
gleich, die Korrektur wandert vom flüchtigen Regler ins bleibende Kennfeld.

> Genau genommen verteilt `Map2d::accumulate` den Betrag bilinear auf vier
> Ecken. Der am selben Punkt *abgetastete* Wert steigt deshalb nur um
> `Σw²·delta` — in der Zellmitte um ein Viertel, exakt auf einem Gitterpunkt um
> den vollen Betrag. Die Zellsumme stimmt, die Sofortwirkung am Betriebspunkt
> ist kleiner. Der Regler holt das nach.

## Die vier Lernpfade

| Pfad | Quelle | Ziel | Achsen |
|---|---|---|---|
| Drosselklappe | Momentenregler | `ecu.throttle_map` | Drehzahl × Momentenwunsch |
| Leerlauf | Leerlaufregler (I-Anteil) | `ecu.idle.trim` | Kühlmitteltemperatur |
| Gemisch | Lambdafehler | `ecu.lambda.trim` | Drehzahl × Last |
| Schaltung | Schlupffehler je Phase | Einkuppelprofil (ILC) | Phase 0…1 |

Dazu der `RlsEstimator`: er schätzt das Moment je Klappenstellung. Das ist keine
Regleranpassung, sondern **Streckenidentifikation** — das Steuergerät lernt den
Motor, nicht sich selbst. Zusehen kann man ihm auf
`adaptation.torque_model.gain`, `.residual` und `.covariance`; sein
Vergessensfaktor ist über `torque_model_forgetting` bedatbar.

## Kurzzeit- und Langzeit-Kraftstofftrimm

Der Kurzzeittrimm ist ein einzelner Skalar. Er reagiert schnell auf den
Lambdafehler, weiß nichts über Betriebspunkte und ist bei jedem Start wieder
null.

Der Langzeittrimm ist **zoniert**: ein grobes Kennfeld über Drehzahl und Last
(`ecu.lambda.trim`, ab Werk 6 × 4 Zonen, alles null). Der Kurzzeittrimm wandert
mit `adaptation.lambda.long_term_rate` hinein, nach demselben erhaltenden Muster
wie oben.

```
adaptation(lambda_long_term_rate: 0.5)
```

**Ab Werk steht die Rate auf `0.0`** — ohne diese Zeile lernt der Langzeittrimm
nicht, und mit einem Nullkennfeld ist der Kraftstofffaktor exakt derselbe wie
vorher. Das ist der Rückfall auf das Standardverhalten.

Auf dem Oszilloskop sind beide Anteile getrennt zu sehen:

| Kanal | Bedeutung |
|---|---|
| `ecu.lambda.short_term` | der flüchtige Anteil, schwingt und läuft gegen null |
| `ecu.lambda.long_term` | der Zonenwert am aktuellen Betriebspunkt |

Der lehrreiche Moment kommt, wenn man in eine Zone fährt, die nie besucht wurde:
dort steht der Langzeitwert auf null und der Kurzzeittrimm muss wieder von vorn
arbeiten.

### Zonen selbst bedaten

Raster, Achsen und Anfangswerte kommen aus dem Skript, wenn man will:

```
engine_control_unit(
    lambda_trim_map: map_2d()
        .add_map_sample(x:  800 * units.rpm, y: 0.0, value: 0.0)
        .add_map_sample(x: 4000 * units.rpm, y: 0.0, value: 0.0)
        .add_map_sample(x:  800 * units.rpm, y: 150 * units.Nm, value: 0.0)
        .add_map_sample(x: 4000 * units.rpm, y: 150 * units.Nm, value: 0.0))
```

Ohne Angabe steht das Standardraster über Drehzahl und Momentenwunsch.

### Lastachse

`lambda_trim_load_manifold: true` zoniert über den Saugrohrdruck statt über den
Momentenwunsch — näher an echten Steuergeräten, die über der relativen
Zylinderfüllung zonieren.

> **Achtung:** der Schalter ändert nur, *welche Größe* abgetastet wird, nicht die
> Achsenwerte des Kennfelds. Die Standardachsen sind über dem Moment gebaut. Wer
> über Saugrohrdruck zoniert, gibt das Kennfeld im Skript vor.

## Lernquelle der Drosselklappenadaption

Ab Werk schöpft die Drosselklappenadaption aus dem **ganzen Reglerausgang**
(P + I + D). Das hat eine unangenehme Folge: ein P-Ausschlag bei einer
Laständerung schreibt in ein bleibendes Kennfeld, und der Integrator wird um
einen Betrag geleert, der nie darin stand — er kann dabei sogar das Vorzeichen
wechseln, obwohl nie etwas hineinlief.

```
adaptation(throttle_learn_from_integrator: true)
```

Damit schöpft sie aus dem I-Anteil, wie der Leerlauftrimm daneben, und die
Buchführung stimmt: abgezogen wird genau, was drin war.

Beide Verhalten sind erreichbar, weil der Unterschied selbst lehrreich ist. Auf
`pid.ecu.torque.i` neben `ecu.throttle_map` sieht man ihn direkt.

## Freigabebedingungen

Adaption läuft nur, wenn der Betriebspunkt sie zulässt. **Alle sieben
Bedingungen sind bedatbar**, im Skript wie im Browser:

| Eingang | Vorgabe |
|---|---|
| `require_warm`, `warm_temperature` | an, 70 °C |
| `require_steady_speed`, `speed_window` | an, 120/min |
| `require_no_shift`, `require_no_limiting` | an |
| `minimum_speed` | 500/min |
| `require_unsaturated_plate` | **aus** |

Der Leerlauftrimm hat zusätzlich eine eigene: er lernt nur, wenn die Drehzahl
wirklich in Leerlaufnähe liegt (`idle_speed_margin`, ab Werk 300/min). Ohne sie
lernte er im Schub bei 3000/min aus einem gesättigten Integrator ins bleibende
Kennfeld.

Für das Schaltlernen gilt davon nur die erste Hälfte der Tabelle:
`require_warm`/`warm_temperature` und `minimum_speed`. Die übrigen vier
beschreiben genau den Zustand, den eine Schaltung selbst herstellt — die
Schaltung *ist* der Schaltvorgang, ihre eigene Momentenrücknahme setzt den Motor
auf `Limiting`, die Drehzahl bricht ein und die Klappe geht an den Anschlag.
Angewandt sperrten sie das Schaltlernen restlos, statt es zu schützen.
`adaptation.shift.enabled` bleibt der Schalter dafür.

Dazu kommt:

```
adaptation(require_unsaturated_plate: true)
```

Die eigentliche Sättigung der Drosselklappe passiert außerhalb des Reglers
(`clamp(vorsteuerung + korrektur, 0, 1)`), die reglereigene
Anti-Windup-Rückführung kennt sie also nicht. Am Anschlag läuft der Integrator
hoch, und die Adaption schriebe diesen Windup ins Kennfeld. Die Bedingung sperrt
das. Ab Werk aus.

## Selbst lernen lassen

Die vier Lernpfade oben sind in C++ verdrahtet. Daneben kann **das Skript
lernen**, auf jedem Pfad des Steuergeräts — aber nur auf denen, die es vorher
ausdrücklich dafür öffnet:

```
set_adaptive(path: "tcu.lockup_map", adaptive: true, min: 0.0, max: 200.0)
```

Ohne diese Zeile verweigert die Registry jeden Lernzugriff. Das ist Absicht: ein
Lernpfad, den niemand freigegeben hat, darf sich nicht selbst öffnen. Ab Werk
offen sind nur die vier eingebauten Kennfelder plus die beiden Schaltkennfelder
(`tcu.upshift_map`, `tcu.downshift_map`, `tcu.lockup_map`) — Schaltpunkt- und
Wandlerüberbrückungsadaption sind reale Getriebefunktionen.

Gelernt wird mit `zone_learner`, und zwar **an der Zelle des Betriebspunkts**:

```
zone_learner(target: "ecu.timing_map",
             error: <block>,
             x:     signal(channel: "engine_speed"),
             y:     signal(channel: "indicated_torque"),
             rate:  0.01)
```

Das ist bilinear verteilt, gegen `min`/`max` geklemmt und damit genau das, was
`updateThrottleMap` und `updateLambdaTrim` in C++ tun. Der ältere `learner`
ohne `x`/`y` lernt weiterhin einzelne Skalare.

Einzelne Zellen sind zusätzlich direkt adressierbar: `ecu.timing_map[3][2]` ist
ein gültiger Pfad für Setzen, Lesen und Lernen.

## Der Zündwinkel

Ab Werk kommt der Frühzündwinkel aus der Kurve des Zündmoduls — eindimensional
über der Drehzahl, so wie es in jedem Motorskript steht. Die ECU legt nur den
Kaltstart-Versatz obendrauf.

```
engine_control_unit(
    timing_map_enabled: true,
    timing_map: map_2d()
        .add_map_sample(x:  800 * units.rpm, y: 0.0, value: 12 * units.deg)
        .add_map_sample(x: 7000 * units.rpm, y: 0.0, value: 34 * units.deg)
        .add_map_sample(x:  800 * units.rpm, y: 200 * units.Nm, value:  8 * units.deg)
        .add_map_sample(x: 7000 * units.rpm, y: 200 * units.Nm, value: 26 * units.deg))
```

Mit dem Schalter kommandiert die ECU den **absoluten** Winkel aus einem Kennfeld
über Drehzahl **und Last** — die Struktur, die ein echtes Steuergerät hat. Der
Kaltstart-Versatz bleibt in beiden Fällen obendrauf.

> **Die Klopfregelung fehlt bewusst.** Sie wäre der eigentliche Lernpfad auf
> diesem Kennfeld, braucht aber eine Klopferkennung, die die Simulation nicht
> hergibt. Das Kennfeld ist echte Bedatung; eine erfundene Klopferkennung wäre
> Fiktion. Wer trotzdem darauf lernen will, kann es über `set_adaptive` und
> `zone_learner` selbst tun — mit einem Fehlersignal eigener Wahl.

## Aktor und Fahrer

Zwei Trägheiten, die vorher fehlten und die Regelaufgabe erst zu einer machen.

**Die Drosselklappe** griff bisher im selben Takt durch. Eine echte E-Gas-Klappe
braucht rund 100 ms über den vollen Weg:

```
set_parameter(path: "throttle.open_rate",     value: 10.0)
set_parameter(path: "throttle.close_rate",    value: 14.0)
set_parameter(path: "throttle.time_constant", value: 0.02)
```

Ab Werk stehen alle drei auf `0` — Durchgriff wie bisher. Wer sie einträgt,
sieht zum ersten Mal, warum der Momentenregler eine Vorsteuerung braucht.

**Das Pedal** wurde bisher in der Grafikdatei geglättet, und zwar pro *Bild*.
Das Pedal, das die ECU sah, hing damit an der Bildrate — und der
Kickdown-Gradient `tcu.kickdown.pedal_rate` ritt darauf. Die Aufbereitung liegt
jetzt im Reglertakt und ist zeitrichtig:

| Pfad | Vorgabe | Bedeutung |
|---|---|---|
| `driver.pedal_time_constant` | `1/60 s` | Trägheit des Gasfußes |
| `driver.clutch_time_constant` | `0.001 s` | Trägheit des Kupplungsfußes |
| `driver.clutch_pedal_rate` | `0.2 1/s` | wie schnell die Kupplung kommt |

Die Vorgabe `1/60 s` trifft exakt das bisherige Verhalten bei 60 Bildern pro
Sekunde — und bleibt danach gleich, egal wie schnell die Grafik läuft.

Auf dem Oszilloskop liegen beide Seiten nebeneinander: `driver.pedal_raw` ist,
was der Fahrer tut, `accelerator` ist, was die ECU sieht.

## Was hier fehlt

- **Klopfregelung.** Siehe oben — es gibt keine Klopferkennung.
- **Persistenz über den Programmstart.** Echte Steuergeräte legen Lernwerte im
  NVRAM ab. Hier sind sie nach dem Beenden weg; `exportScript` schreibt sie als
  Skript heraus.
- **Additiv und multiplikativ getrennt.** Echte Gemischadaption trennt einen
  additiven Anteil (Leerlauf, Nebenluft) von einem multiplikativen (Teillast,
  Einspritzmenge). Hier nur multiplikativ.
- **Einspritzkennfeld je Zylinder.** Die Simulation kennt nur einen globalen
  Kraftstofffaktor.
