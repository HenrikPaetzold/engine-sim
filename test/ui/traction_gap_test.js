// Prueft tractionGap aus assets/config_ui/index.html gegen einen Verlauf, dessen
// Einbruch und Flaeche von Hand ausgerechnet sind.

const fs = require("fs");
const path = require("path");

const source = fs.readFileSync(
    path.join(__dirname, "..", "..", "assets", "config_ui", "index.html"), "utf8");

const begin = source.indexOf("function tractionGap(");
const end = source.indexOf("views.shifts = {");
if (begin < 0 || end < 0 || end <= begin) {
    console.error("tractionGap nicht in index.html gefunden");
    process.exit(1);
}

const tractionGap = new Function(source.slice(begin, end) + "\nreturn tractionGap;")();

let failures = 0;
function check(name, actual, expected, tolerance) {
    const ok = Number.isFinite(actual) && Math.abs(actual - expected) <= tolerance;
    if (!ok) ++failures;
    console.log(
        (ok ? "OK   " : "FAIL ") + name + ": " + Number(actual).toFixed(4)
        + " erwartet " + expected.toFixed(4) + " +-" + tolerance);
}

// Der Rekorder liefert [time, clutch, rpm, request, reduction, slip, outputTorque].
function sample(time, torque) {
    return [time, 0, 0, 0, 0, 0, torque];
}

// 200 Nm, dann 0.1 s lang auf 50 Nm eingebrochen, dann zurueck.
// Einbruch 75 %, Flaeche 150 Nm * 0.1 s = 15 Nm s.
const recording = { samples: [] };
for (let i = 0; i <= 100; ++i) recording.samples.push(sample(i * 0.001, 200));
for (let i = 1; i <= 100; ++i) recording.samples.push(sample(0.1 + i * 0.001, 50));
for (let i = 1; i <= 100; ++i) recording.samples.push(sample(0.2 + i * 0.001, 200));

const gap = tractionGap(recording);
check("Einbruch in Prozent", gap.drop, 75.0, 0.5);
check("verlorene Flaeche", gap.area, 15.0, 0.5);
check("Zeitpunkt des Minimums", gap.lowestTime, 0.101, 0.01);
check("Moment davor", gap.before, 200.0, 0.5);

// Ohne Einbruch ist beides null.
const flat = { samples: [] };
for (let i = 0; i <= 300; ++i) flat.samples.push(sample(i * 0.001, 200));
const none = tractionGap(flat);
check("kein Einbruch", none.drop, 0.0, 0.01);
check("keine Flaeche", none.area, 0.0, 0.01);

// Negatives Moment (Schub) wird ueber den Betrag behandelt.
const coast = { samples: [] };
for (let i = 0; i <= 100; ++i) coast.samples.push(sample(i * 0.001, -120));
for (let i = 1; i <= 100; ++i) coast.samples.push(sample(0.1 + i * 0.001, -30));
for (let i = 1; i <= 100; ++i) coast.samples.push(sample(0.2 + i * 0.001, -120));
check("Schub, Einbruch in Prozent", tractionGap(coast).drop, 75.0, 0.5);

// Zu kurze oder leere Aufnahmen liefern nichts statt Unsinn.
if (tractionGap({ samples: [] }) === null) console.log("OK   leere Aufnahme abgelehnt");
else { console.log("FAIL leere Aufnahme abgelehnt"); ++failures; }

const zero = { samples: [] };
for (let i = 0; i <= 300; ++i) zero.samples.push(sample(i * 0.001, 0));
if (tractionGap(zero) === null) console.log("OK   Nullmoment abgelehnt");
else { console.log("FAIL Nullmoment abgelehnt"); ++failures; }

console.log(failures === 0 ? "\n0 failures" : "\n" + failures + " failures");
process.exit(failures === 0 ? 0 : 1);
