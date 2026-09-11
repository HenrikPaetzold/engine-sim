# Manöver, Aufnahme und Kennzahlen

Ein Manöver ist eine Folge von Stützstellen, die den Fahrer ersetzt: Gas,
Bremse, Kupplung, Wählhebel, Gang, Schaltwünsche — über der Zeit. Man kann es
im Skript schreiben, im Browser starten, oder eine eigene Fahrt aufnehmen und
als Skript zurückbekommen.

Der Zweck ist Vergleichbarkeit. Zwei Bedatungen gegeneinander zu halten geht
nur, wenn beide dieselbe Fahrt fahren.

---

## Ein Manöver schreiben

```
add_manoeuvre(
    manoeuvre(name: "wide open throttle")
        .at(time: 0.0,  accelerator: 0.0, gate: 3)
        .at(time: 1.0,  accelerator: 1.0)
        .at(time: 8.0,  accelerator: 1.0)
        .at(time: 8.2,  accelerator: 0.0, brake: 1.0)
        .at(time: 12.0, accelerator: 0.0, brake: 1.0))
```

Die Datei gehört **nicht** zum Motor. Sie kommt neben den Motor-Import in
`assets/main.mr` — dasselbe Muster wie die Themen. Damit überlebt ein Manöver
den Motorwechsel.

Eingang | Vorgabe | Bedeutung
---|---|---
`time` | — | Sekunden ab Manöverstart
`accelerator` | 0.0 | Fahrpedal, 0…1
`brake` | 0.0 | Bremse, 0…1
`clutch` | 0.0 | **Kupplungspedal**, siehe unten
`gate` | -1 | Wählhebelposition; -1 heißt „nichts wollen"
`gear` | -1 | Gangwunsch im Handmodus
`drive_mode` | 0 | Fahrmodusindex
`manual` | false | Handmodus
`shift_up` / `shift_down` | false | Schaltwunsch, siehe unten
`ignition` | true | Zündschloss
`starter` | false | Anlasser

---

## Drei Fallen, die im Code stehen und die man kennen muss

**1. Die Kupplung ist invertiert.** `clutch: 1.0` heißt Kupplung **offen**, nicht
geschlossen. Beide Verbraucher rechnen `1.0 - clutchPedal`
(`src/powertrain/passthrough_controller.cpp`,
`src/powertrain/transmission_control_unit.cpp`). Die Vorgabe `0.0` heißt „Fuß vom
Pedal", also volle Autorität für das Getriebe — das ist fast immer, was man will.

**2. Stetig wird interpoliert, diskret wird gehalten.** Zwischen zwei
Stützstellen laufen Gas, Bremse und Kupplung linear. Gang, Wählhebel, Fahrmodus
und alle Schalter **springen**. Das ist der Grund, warum ein Manöver kein
`map_2d` ist: ein Kennfeld würde zwischen Gang 3 und 4 den Wert 3.5 liefern und
eine Wählhebelposition auf dem Weg von P nach D durch R und N wischen.

**3. Schaltwünsche sind Impulse, keine Zustände.** Die TCU wertet
`shift_up`/`shift_down` flankengetriggert aus. Der Abspieler setzt sie deshalb
genau **einen Reglertakt** lang, wenn die Stützstelle überschritten wird — man
muss sie nicht von Hand wieder auf `false` ziehen.

---

## Wo das Manöver eingreift

```
Tastatur ──┐
           ├──► m_inputs ──► conditionInputs ──► m_driven ──► ECU / TCU / Adaption
Manöver ───┘                 (Pedalfilter,
                              Kupplungsfilter)
```

Der Abspieler schreibt `m_inputs` **vor** `conditionInputs`
(`src/powertrain_system.cpp`). Damit läuft ein Manöver durch dieselben Filter
wie ein Mensch: kein Sprung, den ein Fahrer nicht auch machen könnte. Für alles
dahinter ist es nicht unterscheidbar.

Die Zeitachse ist `PowertrainSystem::m_time`, also **Simulationszeit**. Ein
Manöver ist damit reproduzierbar und unabhängig von der Bildrate.

---

## Aufnehmen

Der Knopf **record driver inputs** schneidet mit, was der Mensch tut — aus
`driver.pedal_raw` und `driver.clutch_raw`, also **vor** dem Filter. Beim
Stoppen erscheint das fertige `.mr` zum Kopieren.

Beide Stellschrauben der Aufnahme hängen am `driver`-Knoten, weil der Rekorder
den Fahrer aufzeichnet — und beide sind zugleich Schieber im Browser:

```
set_powertrain(
    driver: driver(
        record_interval: 0.02,      // Abtastabstand, Sekunden
        record_tolerance: 0.01))    // Korridorbreite beim Ausduennen
```

Ein kleineres `record_interval` nimmt feiner auf, ein größeres `record_tolerance`
dünnt schärfer aus. Die Registry-Pfade sind `record.interval` und
`record.tolerance`; sie werden beim **Start** einer Aufnahme übernommen, nicht
mittendrin.

Dazwischen liegt das **Ausdünnen**: aus einer Aufnahme mit 50 Abtastungen pro
Sekunde werden wenige Stützstellen. Der Algorithmus ist ein Korridortest — eine
Stützstelle fällt weg, wenn die gerade Linie zwischen ihren Nachbarn überall
innerhalb der Toleranz bleibt. **Jede diskrete Änderung ist eine Zwangsstütze**
und überlebt immer; ein Schaltwunsch geht nie verloren.

Der Rundlauf ist geprüft: das exportierte Skript wird vom echten Compiler
übersetzt und ergibt denselben Verlauf
(`ScriptFixture.ARecordedManoeuvreCompilesAndReplaysTheSameShape`).

---

## Die Grenzen, und wie sie sich melden

Beide Puffer sind endlich, und beide sagen es, wenn sie voll sind — stilles
Abschneiden gibt es nicht.

Grenze | Wert | Was passiert
---|---|---
Stützstellen je Manöver | 512 | Das Skript wird **abgelehnt**, nicht gekürzt, und `error_log.log` nennt Namen und Anzahl
Abtastungen je Aufnahme | 8192 | Die Aufnahme stoppt, die Oberfläche zeigt **BUFFER FULL**, und das exportierte `.mr` trägt eine Kommentarzeile mit der Zahl der verworfenen Proben

Bei 50 Abtastungen pro Sekunde reicht der Aufnahmepuffer für gut zweieinhalb
Minuten. Ein ausgedünntes Manöver liegt weit unter 512 Stützstellen; die Grenze
erreicht man praktisch nur mit einem von Hand geschriebenen Skript oder mit sehr
vielen diskreten Wechseln, weil jeder davon eine Zwangsstütze ist.

Die Kommentarzeile im Export ist Absicht: die Datei wandert weiter, also muss
die Warnung mit ihr wandern. `//` ist in Piranha ein Kommentar, das Skript
bleibt übersetzbar.

---

## Kennzahlen

### Sprungantwort — im Browser, Ansicht „Step"

Rechnet über den Oszilloskop-Puffer, der ohnehin schon mit Reglertakt gefüllt
wird (8 Kanäle, 1024 Punkte).

Kennzahl | Definition
---|---
Anstiegszeit | vom Durchgang durch 10 % bis 90 % des Sprungs
Überschwingen | `(Spitze − Endwert) / Sprunghöhe`, in Prozent
Ausregelzeit | ab Sprungbeginn bis zum letzten Verlassen des ±2-%-Bandes
Endwert | Median des letzten Zehntels

Geprüft gegen analytische Sprungantworten zweiter Ordnung
(`test/ui/step_metrics_test.js`): das Überschwingen stimmt bei drei Dämpfungen
mit `exp(−πζ/√(1−ζ²))` überein.

### Schaltdauer

`tcu.last_shift_duration` und `tcu.shift_elapsed` als Kanäle, dazu eine
Telemetriezeile. Gemessen wird der `StateTimer`, den die TCU ohnehin schon
führt — von `beginShift` bis zum Erreichen von `ShiftState::Idle`.

### Zugkraftloch

Braucht ein Abtriebsmoment, und das gab es vorher nicht. `Transmission::getOutputTorque()`
summiert die **echten Zwangsmomente** der Kupplungen
(`RatioClutchConstraint::getTorque()`), nicht einen Ersatzwert aus Drehzahl und
Übersetzung. Es steht als `state.outputTorque`, als Kanal `output_torque`, in
der Telemetrie und als siebtes Feld in jeder Schaltaufnahme.

Die Auswertung in der Ansicht „Shift scope" nennt den **Einbruch in Prozent**
und die **verlorene Fläche in N·m·s** — letzteres ist der eigentlich
interessante Wert, weil er Tiefe und Dauer zusammenfasst. Gegen einen von Hand
gerechneten Verlauf geprüft (`test/ui/traction_gap_test.js`).

### Warum nicht geschaltet wurde

Von rund dreißig Gründen, aus denen die TCU eine Schaltung ablehnt, waren vier
von außen erkennbar. Jetzt nennt `ShiftBlock` den Grund im Klartext — als
Telemetriezeile **blocked by** und als Zahlkanal `tcu.shift_block` fürs
Oszilloskop.

Grund | heißt
---|---
`NotInDrive` | nicht in einer Fahrstufe
`Reversing` | Rückwärtsgang
`GateRefused` | Wählhebelbewegung verweigert
`ShiftInProgress` | eine Schaltung läuft
`GearDwell` | Mindestgang-Haltezeit nicht abgelaufen
`TopGear` / `BottomGear` | Handmodus am Anschlag
`StallProtection` | Abwürgeschutz
`NoUpshiftThreshold` | unter der Hochschaltlinie
`NoDownshiftThreshold` | über der Rückschaltlinie
`NoKickdownGear` | kein niedrigerer Gang für Kickdown
`Coasting` | Leerlauf in D, Pedal zu

Der Grund wird **gehalten**, nicht gepulst: die TCU läuft mit 1 kHz, die
Telemetrie mit 20 Hz. Ein Grund, der einen Takt lang gesetzt wäre, würde
praktisch nie abgetastet.

**Beachte:** in dem Takt, in dem eine Schaltung *beginnt*, ist der Grund leer —
es wurde ja nichts blockiert. `ShiftInProgress` steht erst im Takt danach.
