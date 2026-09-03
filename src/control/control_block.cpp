#include "../../include/control/control_block.h"

#include "../../include/control/signal_table.h"
#include "../../include/function.h"

#include <algorithm>
#include <cmath>

control::ControlBlock::ControlBlock() {
    m_index = -1;
    m_output = 0.0;
}

control::ControlBlock::~ControlBlock() {
    /* void */
}

void control::ControlBlock::reset() {
    m_output = 0.0;
}

void control::ControlBlock::latch(const BlockContext &context) {
    /* void */
}

bool control::ControlBlock::breaksCycles() const {
    return false;
}

void control::ControlBlock::addOperand(int index) {
    m_operands.push_back(index);
}

double control::ConstantBlock::evaluate(const BlockContext &context) {
    return m_value;
}

double control::SignalBlock::evaluate(const BlockContext &context) {
    if (context.inputs == nullptr) return 0.0;
    return context.inputs->get(m_signal) * m_scale;
}

double control::SumBlock::evaluate(const BlockContext &context) {
    double sum = 0.0;
    for (int i = 0; i < getOperandCount(); ++i) {
        sum += operandValue(context, i);
    }

    return sum;
}

double control::ProductBlock::evaluate(const BlockContext &context) {
    if (getOperandCount() == 0) return 0.0;

    double product = 1.0;
    for (int i = 0; i < getOperandCount(); ++i) {
        product *= operandValue(context, i, 1.0);
    }

    return product;
}

double control::MinBlock::evaluate(const BlockContext &context) {
    if (getOperandCount() == 0) return 0.0;

    double value = operandValue(context, 0);
    for (int i = 1; i < getOperandCount(); ++i) {
        value = std::min(value, operandValue(context, i));
    }

    return value;
}

double control::MaxBlock::evaluate(const BlockContext &context) {
    if (getOperandCount() == 0) return 0.0;

    double value = operandValue(context, 0);
    for (int i = 1; i < getOperandCount(); ++i) {
        value = std::max(value, operandValue(context, i));
    }

    return value;
}

double control::GainBlock::evaluate(const BlockContext &context) {
    return operandValue(context, 0) * m_gain + m_offset;
}

double control::ClampBlock::evaluate(const BlockContext &context) {
    if (m_min > m_max) return operandValue(context, 0);
    return std::clamp(operandValue(context, 0), m_min, m_max);
}

control::CurveBlock::~CurveBlock() {
    if (m_ownsCurve) delete m_curve;
    m_curve = nullptr;
}

void control::CurveBlock::setCurve(Function *curve, bool owned) {
    if (m_ownsCurve) delete m_curve;

    m_curve = curve;
    m_ownsCurve = owned;
}

double control::CurveBlock::evaluate(const BlockContext &context) {
    if (m_curve == nullptr) return operandValue(context, 0);
    return m_curve->sampleTriangle(operandValue(context, 0));
}

double control::MapBlock::evaluate(const BlockContext &context) {
    if (!m_map.isInitialized()) return 0.0;
    return m_map.sample(operandValue(context, 0), operandValue(context, 1));
}

void control::PidBlock::reset() {
    ControlBlock::reset();
    m_controller.reset();
}

double control::PidBlock::evaluate(const BlockContext &context) {
    return m_controller.update(
        context.dt,
        operandValue(context, 0),
        operandValue(context, 1),
        operandValue(context, 2));
}

void control::RateLimitBlock::reset() {
    ControlBlock::reset();
    m_limiter.initialize(m_riseRate, m_fallRate);
}

double control::RateLimitBlock::evaluate(const BlockContext &context) {
    return m_limiter.update(context.dt, operandValue(context, 0));
}

void control::LowPassBlock::reset() {
    ControlBlock::reset();
    m_state = 0.0;
    m_primed = false;
}

double control::LowPassBlock::evaluate(const BlockContext &context) {
    const double target = operandValue(context, 0);

    if (!m_primed) {
        m_primed = true;
        m_state = target;
        return m_state;
    }

    if (m_timeConstant <= 0.0) {
        m_state = target;
        return m_state;
    }

    const double alpha = context.dt / (m_timeConstant + context.dt);
    m_state += alpha * (target - m_state);

    return m_state;
}

double control::SelectBlock::evaluate(const BlockContext &context) {
    return (operandValue(context, 0) >= m_threshold)
        ? operandValue(context, 1)
        : operandValue(context, 2);
}

void control::CompareBlock::reset() {
    ControlBlock::reset();
    m_hysteresis.setState(false);
}

double control::CompareBlock::evaluate(const BlockContext &context) {
    const double difference = operandValue(context, 0) - operandValue(context, 1);

    m_hysteresis.initialize(-m_band, m_band, m_hysteresis.getState());
    return m_hysteresis.update(difference) ? 1.0 : 0.0;
}

void control::LatchBlock::reset() {
    ControlBlock::reset();
    m_state = false;
}

double control::LatchBlock::evaluate(const BlockContext &context) {
    if (operandValue(context, 1) >= m_threshold) m_state = false;
    else if (operandValue(context, 0) >= m_threshold) m_state = true;

    return m_state ? 1.0 : 0.0;
}

void control::IntegratorBlock::reset() {
    ControlBlock::reset();
    m_value = m_initial;
}

double control::IntegratorBlock::evaluate(const BlockContext &context) {
    m_value += operandValue(context, 0) * context.dt;
    m_value = std::clamp(m_value, m_min, m_max);

    return m_value;
}

void control::TimerBlock::reset() {
    ControlBlock::reset();
    m_elapsed = 0.0;
}

double control::TimerBlock::evaluate(const BlockContext &context) {
    if (operandValue(context, 0) >= m_threshold) m_elapsed += context.dt;
    else m_elapsed = 0.0;

    return m_elapsed;
}

void control::DelayBlock::reset() {
    ControlBlock::reset();
    m_previous = m_initial;
}

bool control::DelayBlock::breaksCycles() const {
    return true;
}

double control::DelayBlock::evaluate(const BlockContext &context) {
    return m_previous;
}

void control::DelayBlock::latch(const BlockContext &context) {
    m_previous = operandValue(context, 0);
}

double control::ActuatorBlock::evaluate(const BlockContext &context) {
    const double value = operandValue(context, 0);
    if (context.outputs != nullptr) context.outputs->set(m_actuator, value);

    return value;
}
