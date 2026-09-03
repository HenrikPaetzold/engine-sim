#include "../include/powertrain_cluster.h"

#include "../include/engine_sim_application.h"
#include "../include/units.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

PowertrainCluster::PowertrainCluster() {
    m_simulator = nullptr;

    m_torqueRequestScope = nullptr;
    m_torqueActualScope = nullptr;
    m_throttleScope = nullptr;
    m_pedalScope = nullptr;
    m_clutchScope = nullptr;
    m_slipScope = nullptr;

    for (int i = 0; i < ShiftHistory; ++i) {
        m_shiftQuality[i] = 0.0;
    }

    m_shiftCount = 0;
    m_lastShiftIteration = 0;
}

PowertrainCluster::~PowertrainCluster() {
    /* void */
}

void PowertrainCluster::initialize(EngineSimApplication *app) {
    UiElement::initialize(app);

    m_torqueRequestScope = addElement<Oscilloscope>(this);
    m_torqueActualScope = addElement<Oscilloscope>(this);
    m_throttleScope = addElement<Oscilloscope>(this);
    m_pedalScope = addElement<Oscilloscope>(this);
    m_clutchScope = addElement<Oscilloscope>(this);
    m_slipScope = addElement<Oscilloscope>(this);

    Oscilloscope *const scopes[] = {
        m_torqueRequestScope,
        m_torqueActualScope,
        m_throttleScope,
        m_pedalScope,
        m_clutchScope,
        m_slipScope };

    for (Oscilloscope *scope : scopes) {
        scope->setBufferSize(512);
        scope->m_xMin = 0.0;
        scope->m_xMax = 10.0;
        scope->m_lineWidth = 2.0f;
        scope->m_drawReverse = false;
        scope->m_drawZero = true;
    }

    m_torqueRequestScope->m_yMin = 0.0;
    m_torqueRequestScope->m_yMax = units::torque(400.0, units::Nm);
    m_torqueRequestScope->m_dynamicallyResizeY = true;
    m_torqueRequestScope->i_color = m_app->getPink();

    m_torqueActualScope->m_yMin = 0.0;
    m_torqueActualScope->m_yMax = units::torque(400.0, units::Nm);
    m_torqueActualScope->m_dynamicallyResizeY = true;
    m_torqueActualScope->i_color = m_app->getOrange();

    m_throttleScope->m_yMin = 0.0;
    m_throttleScope->m_yMax = 1.0;
    m_throttleScope->i_color = m_app->getOrange();

    m_pedalScope->m_yMin = 0.0;
    m_pedalScope->m_yMax = 1.0;
    m_pedalScope->i_color = m_app->getBlue();

    m_clutchScope->m_yMin = 0.0;
    m_clutchScope->m_yMax = 1.0;
    m_clutchScope->i_color = m_app->getOrange();

    m_slipScope->m_yMin = 0.0;
    m_slipScope->m_yMax = units::rpm(3000.0);
    m_slipScope->m_dynamicallyResizeY = true;
    m_slipScope->i_color = m_app->getPink();
}

void PowertrainCluster::destroy() {
    UiElement::destroy();
}

void PowertrainCluster::update(float dt) {
    UiElement::update(dt);
}

void PowertrainCluster::sample() {
    if (m_simulator == nullptr) return;
    if (!m_simulator->m_powertrain.isActive()) return;

    m_simulator->m_powertrain.fillTelemetry(&m_sample);

    const powertrain::DriverInputs &inputs =
        m_simulator->m_powertrain.getDriverInputs();
    const powertrain::PowertrainState &state =
        m_simulator->m_powertrain.getState();

    const double t = m_sample.time;

    m_torqueRequestScope->addDataPoint(t, m_sample.torqueRequest);
    m_torqueActualScope->addDataPoint(t, m_sample.indicatedTorque);
    m_throttleScope->addDataPoint(t, m_sample.throttlePlate);
    m_pedalScope->addDataPoint(t, inputs.accelerator);
    m_clutchScope->addDataPoint(t, m_sample.clutchPressure);
    m_slipScope->addDataPoint(t, std::abs(state.clutchSlipSpeed[0]));

    Oscilloscope *const scopes[] = {
        m_torqueRequestScope,
        m_torqueActualScope,
        m_throttleScope,
        m_pedalScope,
        m_clutchScope,
        m_slipScope };

    for (Oscilloscope *scope : scopes) {
        scope->m_xMax = t;
        scope->m_xMin = t - 10.0;
    }

    if (m_sample.shiftIterations > m_lastShiftIteration) {
        m_lastShiftIteration = m_sample.shiftIterations;

        m_shiftQuality[m_shiftCount % ShiftHistory] = m_sample.shiftErrorNorm;
        ++m_shiftCount;
    }
}

void PowertrainCluster::renderScope(
    Oscilloscope *scope,
    const Bounds &bounds,
    const std::string &title,
    bool overlay)
{
    if (!overlay) {
        drawFrame(bounds, 1.0f, m_app->getForegroundColor(), m_app->getBackgroundColor());
    }

    scope->render(bounds);

    if (!title.empty()) {
        drawText(title, bounds.inset(10.0f), 16.0f, Bounds::tl);
    }
}

std::string PowertrainCluster::gearLabel() const {
    if (m_sample.range == "P" || m_sample.range == "N") return m_sample.range;
    if (m_sample.range == "R") return "R";
    if (m_sample.gear < 0) return "N";

    return std::to_string(m_sample.gear + 1);
}

void PowertrainCluster::renderStatus(const Bounds &bounds) {
    drawFrame(bounds, 1.0f, m_app->getForegroundColor(), m_app->getBackgroundColor());

    Grid grid;
    grid.h_cells = 1;
    grid.v_cells = 9;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(0);

    ss << "ENGINE  " << m_sample.engineState;
    drawText(ss.str(), grid.get(bounds, 0, 0).inset(10.0f), 16.0f, Bounds::tl);

    ss.str("");
    ss << "SHIFT   " << m_sample.shiftState;
    drawText(ss.str(), grid.get(bounds, 0, 1).inset(10.0f), 16.0f, Bounds::tl);

    ss.str("");
    ss << "SELECT  " << m_sample.range;
    drawText(ss.str(), grid.get(bounds, 0, 2).inset(10.0f), 16.0f, Bounds::tl);

    ss.str("");
    ss << "GEAR    " << gearLabel();
    drawText(ss.str(), grid.get(bounds, 0, 3).inset(10.0f), 16.0f, Bounds::tl);

    ss.str("");
    ss << "SPEED   "
        << units::convert(std::abs(m_sample.vehicleSpeed), units::km / units::hour)
        << " KPH";
    drawText(ss.str(), grid.get(bounds, 0, 4).inset(10.0f), 16.0f, Bounds::tl);

    ss.str("");
    ss << "COOLANT "
        << (m_sample.coolantTemperature - units::celcius(0.0)) << " C";
    drawText(ss.str(), grid.get(bounds, 0, 5).inset(10.0f), 16.0f, Bounds::tl);

    ss.str("");
    ss << "OIL     "
        << (m_sample.oilTemperature - units::celcius(0.0)) << " C";
    drawText(ss.str(), grid.get(bounds, 0, 6).inset(10.0f), 16.0f, Bounds::tl);

    ss.str("");
    ss << std::setprecision(2);
    ss << "GRADE   " << (m_sample.roadGrade * 100.0) << " %";
    drawText(ss.str(), grid.get(bounds, 0, 7).inset(10.0f), 16.0f, Bounds::tl);

    ss.str("");
    ss << "ADAPT   " << (m_sample.adaptionEnabled ? "LEARNING" : "HELD");
    drawText(ss.str(), grid.get(bounds, 0, 8).inset(10.0f), 16.0f, Bounds::tl);
}

void PowertrainCluster::renderShiftQuality(const Bounds &bounds) {
    drawFrame(bounds, 1.0f, m_app->getForegroundColor(), m_app->getBackgroundColor());
    drawText("SHIFT QUALITY", bounds.inset(10.0f), 16.0f, Bounds::tl);

    const int shown = std::min(m_shiftCount, ShiftHistory);
    if (shown == 0) return;

    double peak = 0.0;
    for (int i = 0; i < shown; ++i) {
        peak = std::max(peak, m_shiftQuality[i]);
    }

    if (peak <= 0.0) return;

    const Bounds plot = bounds.inset(20.0f);
    const float width = plot.width() / static_cast<float>(ShiftHistory);

    const int first = std::max(0, m_shiftCount - ShiftHistory);
    for (int i = 0; i < shown; ++i) {
        const double value = m_shiftQuality[(first + i) % ShiftHistory];
        const float height =
            static_cast<float>(value / peak) * plot.height();

        const float x0 = plot.left() + i * width;

        Bounds bar(
            width * 0.7f,
            height,
            { x0, plot.bottom() },
            Bounds::bl);

        drawBox(bar, m_app->getOrange());
    }
}

void PowertrainCluster::render() {
    Grid grid;
    grid.h_cells = 3;
    grid.v_cells = 2;

    renderScope(m_torqueRequestScope, grid.get(m_bounds, 0, 0), "Torque request / actual");
    renderScope(m_torqueActualScope, grid.get(m_bounds, 0, 0), "", true);

    renderScope(m_pedalScope, grid.get(m_bounds, 1, 0), "Pedal / throttle plate");
    renderScope(m_throttleScope, grid.get(m_bounds, 1, 0), "", true);

    renderStatus(grid.get(m_bounds, 2, 0));

    renderScope(m_clutchScope, grid.get(m_bounds, 0, 1), "Clutch pressure");
    renderScope(m_slipScope, grid.get(m_bounds, 1, 1), "Clutch slip");

    renderShiftQuality(grid.get(m_bounds, 2, 1));

    UiElement::render();
}
