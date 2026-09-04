# Adaption im Skript · Kalibrierung im Browser

## Adaption ist keine C++-Sache mehr

Vorher waren genau drei Parameter adaptiv, alle fest verdrahtet: `ecu.throttle_map`,
`ecu.idle.trim`, `tcu.upshift_map`. Ein Block im Blockschaltbild wurde als einfacher
Skalar registriert und vom `AdaptationManager` nie angefasst — Stufe B konnte regeln,
aber nicht lernen.

Jetzt deklariert das Skript beides selbst.

### Ein Block kann adaptiv sein

```
gain(name: "pedal", a: signal(channel: "accelerator"),
     gain: 1.0, adaptive: true, adapt_min: 0.5, adapt_max: 1.5)
```

Die Flaggen wandern in den `ParameterDescriptor`. Damit greift die vorhandene
Schutzlogik unverändert: `ParameterRegistry::adapt()` verweigert nicht-adaptive Pfade
und klemmt gegen `adaptMin`/`adaptMax`.

### Der `learner`-Block

```
learner(target: "program.pedal.gain",
        error:  <block>,
        rate:   0.05,
        enable: <block>)
```

Pro Takt: `registry.adapt(target, −rate · error · dt)`, aber nur wenn `enable ≥ 0.5`.
Ausgang ist der aktuelle Zielwert, damit er plottbar und rückkoppelbar ist.

Weil das Ziel ein **Registry-Pfad** ist, erreicht der Lerner auch ECU- und
TCU-Parameter, nicht nur eigene Blöcke — dieselbe universelle Schreibfläche, die
Fahrmodi, Browser und Export benutzen. Freigabebedingungen baust du aus
`greater_than`, `timer` und `latch`, genau wie der `AdaptationManager` es in C++ tut:

```
learner(target: "ecu.idle.trim", rate: 0.02,
        error:  sum(a: signal(channel: "engine_speed"), b: gain(a: idle_target, gain: -1)),
        enable: greater_than(a: signal(channel: "coolant_temperature"),
                             b: constant(80 + units.K0), band: 2))
```

Ohne Registry ist der Block ein harmloses No-Op — geprüft.

### Fix und adaptiv zur Laufzeit

Jede Parameterzeile im Browser hat ein Häkchen. `POST /api/adaptive` geht durch
dieselbe threadsichere Kommandoqueue wie `/api/set`. Damit kannst du einen Lerner
einfrieren, von Hand einen anderen Wert setzen und wieder freigeben — und zusehen,
wie er ihn erneut einfängt.

---

## Der Wählhebel liegt auf den Pfeiltasten

`O` und `P` sind weg. `SelectorGate::isDefault()` unterscheidet eine geskriptete
Gasse von der automatisch erzeugten:

| Lage | Hoch / Runter |
|---|---|
| Geskriptete Gasse **und** Automatikmodus | eine Raste durch die Gasse |
| Handmodus (`J`) | Gänge tippen, Raste bleibt stehen |
| Keine geskriptete Gasse | Gänge, wie immer |

`J` ist der Kipphebel in die M-Gasse. `K` bremst weiterhin.

---

## Der Drehzahlmesser zeigt, was die ECU denkt

Vorher leiteten sich alle drei Bänder aus **einer** statischen Zahl ab
(`Engine::getRedline()`), das orange Band aus einer 0,9-Faustregel ohne
physikalische Bedeutung.

Jetzt kommandiert die ECU beide Schwellen:

```
softLimitStart = effektiveRevLimit − softLimitBand    // Zündung blendet aus
revLimit       = effektiveRevLimit + hardLimitOffset  // harter Kraftstoff-Cut
```

und der Drehzahlmesser folgt ihnen mit 1,5 s Zeitkonstante, damit ein Moduswechsel
als Gleiten sichtbar wird statt als Sprung. **Die Skala bleibt beim Skriptwert** —
eine bestimmte Drehzahl liegt damit immer an derselben Stelle, während die Bänder
wandern. Ohne Steuergerät bleibt alles wie zuvor.

### Die kalte Drehzahlgrenze

`revLimitCold` interpoliert über dieselbe `warmupFraction`, die schon
Leerlaufdrehzahl, Anfettung, Zündwinkel und Momentendeckel steuert:

```
effektiveRevLimit = revLimitCold + (revLimit − revLimitCold) · warm
```

Beim Kaltstart sitzt der Begrenzer damit wirklich tiefer und wandert über die
Warmlaufphase nach oben — sichtbar am Zifferblatt.

---

## Fünf Kalibrierungsansichten im Browser

Die Kennfelddaten lagen längst auf der Leitung: `/api/schema` überträgt für alle
sechs Maps Achsen *und* Werte. Die Oberfläche hat sie an einer Zeile weggeworfen.

| Ansicht | Was sie beantwortet |
|---|---|
| **Shift map** | Geschwindigkeit × Pedal, Hochschaltlinien durchgezogen, Rückschaltlinien gestrichelt, eine Farbe je Gang. Der Abstand eines Paares **ist** die Hysterese. Sport schiebt die Schar nach rechts, Eco nach links. |
| **Throttle map** | Drehzahl × Sollmoment als Heatmap, Farbe ist die Klappe. Umschaltbar auf **Delta**: Differenz zum ersten gesehenen Kennfeld, divergierende Skala — da siehst du dem RLS beim Lernen zu. |
| **Pedal** | Pedalweg gegen Momentenanteil. **Halten** friert die Kurve ein, dann Modus wechseln und vergleichen. |
| **Operating points** | Drehzahl × Moment, über die Fahrt akkumuliert, nach Gang eingefärbt. Die Ökonomie-Ansicht. |
| **Shift scope** | Die letzten 8 Schaltungen auf einer Zeitbasis, ältere blasser. Kupplungsdruck und Schlupf. Darin sieht man das ILC konvergieren. |

### Warum das Schalt-Oszilloskop C++ braucht

20 Hz Telemetrie sind sechs Punkte pro Schaltung. `config::ShiftRecorder` schreibt
deshalb im Reglertakt mit, ausgelöst von der steigenden Flanke von
`PowertrainBus::shiftInProgress`, und dezimiert auf **512 Stützstellen je Aufnahme**
(≈ 340 Hz) bei einem Ring der **letzten 8 Aufnahmen**. Damit ist der Speicher fest
und die JSON-Größe beherrschbar.

Das Fenster ist bewusst eine feste Dauer ab Schaltbeginn, nicht bis Schaltende —
sonst hätten die Aufnahmen keine gemeinsame Zeitbasis und ließen sich nicht
überlagern.

Übertragen über `GET /api/shifts`, serialisiert im `publish()`-Pfad in einen
vorbereiteten String — dasselbe Muster wie `/api/export`, damit der Serverthread die
Simulationsdaten nie direkt liest.

## Was live läuft

Die Trennlinie ist bewusst gezogen:

| | Inhalt | Takt |
|---|---|---|
| `/api/schema` | nur **Unveränderliches**: Pfade, Typ, Einheit, Grenzen, Vorgabewert, Kennfeld-Achsen | einmal beim Laden |
| `/api/state` | alles **Veränderliche**: Skalarwerte, **Kennfeldwerte**, **Adaptiv-Kennzeichen**, Telemetrie | 100 ms |
| `/api/shifts` | die letzten acht Schaltaufzeichnungen | 2 s |
| `/api/export` `/api/overrides` | gelernte bzw. geänderte Werte als Skript | 2 s |

Damit ist per Konstruktion nichts eingefroren: was sich ändern kann, liegt im
Live-Pfad; was im Schema steht, kann sich nicht ändern.

Vorher schrieb `refreshState()` nur die skalaren Werte und übersprang jedes
Kennfeld; die Kennfeldwerte standen ausschließlich im Schema, das genau einmal
beim Start gebaut wurde. Die Delta-Ansicht des Drosselklappen-Kennfelds
verglich dadurch zwei Kopien desselben eingefrorenen Zustands und konnte
nichts anzeigen — genau die Ansicht, die dem Schätzer beim Lernen zusehen soll.

Alle Kennfelder zusammen sind rund 310 Werte, also etwa vier Kilobyte Text je
Abruf. Über localhost ist das belanglos.

## Zellen bearbeiten

Unter der Zeichnung steht in jeder Kennfeldansicht ein Editor: Kennfeld
auswählen, Spalte und Zeile wählen, Wert eingeben. Geschrieben wird über
`POST /api/set` mit dem Zellenpfad, und der neue Wert kommt über denselben
Live-Pfad zurück — die Zeichnung zieht also im nächsten Takt nach, ohne
Sonderweg.

## Kennfelder als Overlay

Bis hierher war die ganze Überschreibungskette skalar: `DriveMode::set()` hielt
einen `double` je Pfad, `ParameterRegistry::set()` schrieb einen `double`, und
der Server nahm `{path, value}`. Kennfelder waren zwar registriert, aber von
keiner Seite schreibbar — ein Fahrmodus konnte also keinen anderen Schaltplan
auflegen.

Die Registry löst zusätzlich Pfade mit Zellensuffix auf:

```
tcu.upshift_map[3][2]        Spalte 3, Zeile 2
```

Der Zugriff ist rein additiv. Erst wenn die normale Suche in der Hashtabelle
fehlschlägt, wird ein Suffix geprüft, der Basispfad aufgelöst und als Kennfeld
verifiziert. Bestehende Pfade laufen unverändert. Daraus fällt alles Weitere
ohne neue Mechanik an: Fahrmodi tragen Kennfelder, `POST /api/set` erreicht
Zellen, und die Wiederherstellung gegen die Grundlinie beim Moduswechsel gilt
für Zellen wie für Skalare.

### Ganze Kennfelder im Skript

Zellenweise Overrides sind für einen 12×6-Schaltplan unbrauchbar — 72 Zeilen —
und ein Skript bedatet ohnehin in physikalischen Größen, nicht in Rasterindizes.
`set_map` trägt deshalb das Kennfeld selbst:

```
add_drive_mode(drive_mode(name: "eco")
    .set_map(path: "tcu.upshift_map", map: map_2d()
        .add_map_sample(x: 0.0, y: 0.0, value: 8.0)
        .add_map_sample(x: 1.0, y: 0.0, value: 22.0)))
```

Aufgelöst wird es erst beim Umschalten: das Kennfeld des Skripts wird auf den
Achsen des lebenden Kennfelds abgetastet und in Zellen-Overrides expandiert.
Die Rasterung des Ziels darf sich damit ändern, ohne dass das Skript etwas davon
weiß, und `DriveMode` bleibt intern skalar.

### Export und Wiedereinlesen

`ParameterRegistry::exportScript` schrieb schon immer `set_parameter(...)` und
`set_map_cell(...)`, aber beide Knoten gab es in der Sprache nicht — eine
exportierte Datei mit gelernten Werten war nicht kompilierbar. Jetzt gibt es
sie, und der Kreis schließt sich: exportieren, ins Skript legen, wieder
einlesen.
