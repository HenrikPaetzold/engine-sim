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

Was rausfließt, fließt rein. Die Summe bleibt gleich, die Korrektur wandert vom
flüchtigen Regler ins bleibende Kennfeld — und wirkt dort ohne Regelabweichung.

## Die vier Lernpfade

| Pfad | Quelle | Ziel | Achsen |
|---|---|---|---|
| Drosselklappe | Momentenregler | `ecu.throttle_map` | Drehzahl × Momentenwunsch |
| Leerlauf | Leerlaufregler (I-Anteil) | `ecu.idle.trim` | Kühlmitteltemperatur |
| Gemisch | Lambdafehler | `ecu.lambda.trim` | Drehzahl × Last |
| Schaltung | Schlupffehler je Phase | Einkuppelprofil (ILC) | Phase 0…1 |

Dazu der `RlsEstimator`: er schätzt das Moment je Klappenstellung. Das ist keine
Regleranpassung, sondern **Streckenidentifikation** — das Steuergerät lernt den
Motor, nicht sich selbst.

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

Adaption läuft nur, wenn der Betriebspunkt sie zulässt — warm, keine Schaltung,
kein Begrenzer, stationäre Drehzahl. Dazu kommt:

```
adaptation(require_unsaturated_plate: true)
```

Die eigentliche Sättigung der Drosselklappe passiert außerhalb des Reglers
(`clamp(vorsteuerung + korrektur, 0, 1)`), die reglereigene
Anti-Windup-Rückführung kennt sie also nicht. Am Anschlag läuft der Integrator
hoch, und die Adaption schriebe diesen Windup ins Kennfeld. Die Bedingung sperrt
das. Ab Werk aus.

## Was hier fehlt

- **Persistenz über den Programmstart.** Echte Steuergeräte legen Lernwerte im
  NVRAM ab. Hier sind sie nach dem Beenden weg; `exportScript` schreibt sie als
  Skript heraus.
- **Additiv und multiplikativ getrennt.** Echte Gemischadaption trennt einen
  additiven Anteil (Leerlauf, Nebenluft) von einem multiplikativen (Teillast,
  Einspritzmenge). Hier nur multiplikativ.
