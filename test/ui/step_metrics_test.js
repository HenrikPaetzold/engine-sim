// Prueft stepMetrics aus assets/config_ui/index.html gegen analytisch bekannte
// Sprungantworten. Laeuft ohne Browser: node test/ui/step_metrics_test.js

const fs = require("fs");
const path = require("path");

const source = fs.readFileSync(
    path.join(__dirname, "..", "..", "assets", "config_ui", "index.html"), "utf8");

const begin = source.indexOf("function stepMetrics(");
const end = source.indexOf("views.step = {");
if (begin < 0 || end < 0 || end <= begin) {
    console.error("stepMetrics nicht in index.html gefunden");
    process.exit(1);
}

const stepMetrics = new Function(source.slice(begin, end) + "\nreturn stepMetrics;")();

let failures = 0;
function check(name, actual, expected, tolerance) {
    const ok = Number.isFinite(actual) && Math.abs(actual - expected) <= tolerance;
    if (!ok) ++failures;
    console.log(
        (ok ? "OK   " : "FAIL ") + name + ": " + Number(actual).toFixed(4)
        + " erwartet " + expected.toFixed(4) + " +-" + tolerance);
}

function secondOrder(damping, frequency, duration, step) {
    const time = [];
    const value = [];
    const damped = frequency * Math.sqrt(1 - damping * damping);

    for (let t = 0; t <= duration; t += step) {
        time.push(t);
        if (t < 0.5) { value.push(0.0); continue; }
        const since = t - 0.5;
        value.push(1 - Math.exp(-damping * frequency * since)
            * (Math.cos(damped * since)
                + (damping * frequency / damped) * Math.sin(damped * since)));
    }

    return { time, value };
}

for (const damping of [0.3, 0.5, 0.7]) {
    const frequency = 10.0;
    const run = secondOrder(damping, frequency, 4.0, 0.001);
    const metrics = stepMetrics(run.time, run.value);

    check("overshoot d=" + damping, metrics.overshoot,
        Math.exp(-Math.PI * damping / Math.sqrt(1 - damping * damping)) * 100, 1.0);
    check("rise time d=" + damping, metrics.riseTime,
        (2.16 * damping + 0.60) / frequency, 0.012);
    check("settling time d=" + damping, metrics.settleTime,
        4.0 / (damping * frequency), 0.5);
}

const tau = 0.05;
const first = { time: [], value: [] };
for (let t = 0; t <= 2.0; t += 0.001) {
    first.time.push(t);
    first.value.push(t < 0.5 ? 0.0 : 1 - Math.exp(-(t - 0.5) / tau));
}

const firstOrder = stepMetrics(first.time, first.value);
check("first order overshoot", firstOrder.overshoot, 0.0, 0.2);
check("first order rise time", firstOrder.riseTime, tau * Math.log(9), 0.003);

const flat = stepMetrics(first.time, first.time.map(() => 7.0));
if (flat && flat.flat) console.log("OK   flat signal recognised");
else { console.log("FAIL flat signal recognised"); ++failures; }

const falling = stepMetrics(first.time, first.value.map(v => 1.0 - v));
check("falling step rise time", falling.riseTime, tau * Math.log(9), 0.003);

const short = stepMetrics([0, 1], [0, 1]);
if (short === null) console.log("OK   too few samples rejected");
else { console.log("FAIL too few samples rejected"); ++failures; }

console.log(failures === 0 ? "\n0 failures" : "\n" + failures + " failures");
process.exit(failures === 0 ? 0 : 1);
