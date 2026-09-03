#include "../../include/powertrain/selector_gate.h"

#include <algorithm>

namespace {
    const powertrain::GatePosition s_neutral;
}

powertrain::SelectorGate::SelectorGate() {
    /* void */
}

powertrain::SelectorGate::~SelectorGate() {
    /* void */
}

void powertrain::SelectorGate::clear() {
    m_positions.clear();
}

void powertrain::SelectorGate::add(const GatePosition &position) {
    m_positions.push_back(position);
}

void powertrain::SelectorGate::buildDefault() {
    clear();

    GatePosition park;
    park.name = "P";
    park.engagement = GateEngagement::Park;
    park.maxEntrySpeed = 0.3;
    park.maxExitSpeed = 0.3;
    park.requiresBrake = true;
    add(park);

    GatePosition reverse;
    reverse.name = "R";
    reverse.engagement = GateEngagement::Reverse;
    reverse.maxEntrySpeed = 1.0;
    add(reverse);

    GatePosition neutral;
    neutral.name = "N";
    neutral.engagement = GateEngagement::Neutral;
    add(neutral);

    GatePosition drive;
    drive.name = "D";
    drive.engagement = GateEngagement::Forward;
    add(drive);
}

int powertrain::SelectorGate::getCount() const {
    return static_cast<int>(m_positions.size());
}

const powertrain::GatePosition &powertrain::SelectorGate::get(int index) const {
    if (index < 0 || index >= getCount()) return s_neutral;
    return m_positions[index];
}

int powertrain::SelectorGate::find(const std::string &name) const {
    for (int i = 0; i < getCount(); ++i) {
        if (m_positions[i].name == name) return i;
    }

    return -1;
}

int powertrain::SelectorGate::clampIndex(int index) const {
    if (getCount() == 0) return 0;
    return std::clamp(index, 0, getCount() - 1);
}
