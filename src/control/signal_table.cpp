#include "../../include/control/signal_table.h"

namespace {
    const std::string s_empty;
}

control::SignalTable::SignalTable() {
    /* void */
}

control::SignalTable::~SignalTable() {
    /* void */
}

int control::SignalTable::declare(const std::string &name) {
    const int existing = find(name);
    if (existing != -1) return existing;

    m_names.push_back(name);
    m_values.push_back(0.0);

    return static_cast<int>(m_names.size()) - 1;
}

int control::SignalTable::find(const std::string &name) const {
    for (size_t i = 0; i < m_names.size(); ++i) {
        if (m_names[i] == name) return static_cast<int>(i);
    }

    return -1;
}

void control::SignalTable::clearValues() {
    for (size_t i = 0; i < m_values.size(); ++i) {
        m_values[i] = 0.0;
    }
}

void control::SignalTable::set(int index, double value) {
    if (index < 0 || index >= static_cast<int>(m_values.size())) return;
    m_values[index] = value;
}

double control::SignalTable::get(int index) const {
    if (index < 0 || index >= static_cast<int>(m_values.size())) return 0.0;
    return m_values[index];
}

const std::string &control::SignalTable::getName(int index) const {
    if (index < 0 || index >= static_cast<int>(m_names.size())) return s_empty;
    return m_names[index];
}
