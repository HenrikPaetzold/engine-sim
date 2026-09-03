#include "../../include/control/iterative_learning.h"

#include <algorithm>
#include <cassert>
#include <cmath>

control::IterativeLearningControl::IterativeLearningControl() {
    m_profile = nullptr;
    m_errorSum = nullptr;
    m_errorCount = nullptr;
    m_lastErrorNorm = 0.0;
    m_iterationCount = 0;
    m_recording = false;
}

control::IterativeLearningControl::~IterativeLearningControl() {
    destroy();
}

void control::IterativeLearningControl::initialize(const Parameters &params) {
    destroy();

    m_params = params;
    m_params.binCount = std::max(m_params.binCount, 1);

    m_profile = new double[m_params.binCount];
    m_errorSum = new double[m_params.binCount];
    m_errorCount = new int[m_params.binCount];

    reset();
}

void control::IterativeLearningControl::destroy() {
    delete[] m_profile;
    delete[] m_errorSum;
    delete[] m_errorCount;

    m_profile = nullptr;
    m_errorSum = nullptr;
    m_errorCount = nullptr;
}

void control::IterativeLearningControl::reset() {
    if (m_profile == nullptr) return;

    for (int i = 0; i < m_params.binCount; ++i) {
        m_profile[i] = 0.0;
        m_errorSum[i] = 0.0;
        m_errorCount[i] = 0;
    }

    m_lastErrorNorm = 0.0;
    m_iterationCount = 0;
    m_recording = false;
}

int control::IterativeLearningControl::binOf(double phase) const {
    const double clamped = std::clamp(phase, 0.0, 1.0);
    const int bin = static_cast<int>(clamped * m_params.binCount);

    return std::min(bin, m_params.binCount - 1);
}

void control::IterativeLearningControl::beginIteration() {
    if (m_profile == nullptr) return;

    for (int i = 0; i < m_params.binCount; ++i) {
        m_errorSum[i] = 0.0;
        m_errorCount[i] = 0;
    }

    m_recording = true;
}

void control::IterativeLearningControl::sample(double phase, double error) {
    if (m_profile == nullptr || !m_recording) return;

    const int bin = binOf(phase);
    m_errorSum[bin] += error;
    ++m_errorCount[bin];
}

void control::IterativeLearningControl::discardIteration() {
    m_recording = false;
}

void control::IterativeLearningControl::endIteration() {
    if (m_profile == nullptr || !m_recording) return;

    m_recording = false;

    double norm = 0.0;
    int observed = 0;

    for (int i = 0; i < m_params.binCount; ++i) {
        if (m_errorCount[i] == 0) continue;

        const double mean = m_errorSum[i] / m_errorCount[i];
        norm += mean * mean;
        ++observed;

        m_profile[i] = std::clamp(
            m_profile[i] + m_params.learningRate * mean,
            m_params.outputMin,
            m_params.outputMax);
    }

    if (observed > 0) m_lastErrorNorm = std::sqrt(norm / observed);

    if (m_params.smoothing > 0.0 && m_params.binCount > 2) {
        double *smoothed = new double[m_params.binCount];

        for (int i = 0; i < m_params.binCount; ++i) {
            const int previous = std::max(i - 1, 0);
            const int next = std::min(i + 1, m_params.binCount - 1);
            const double neighbours = 0.5 * (m_profile[previous] + m_profile[next]);

            smoothed[i] =
                (1.0 - m_params.smoothing) * m_profile[i]
                + m_params.smoothing * neighbours;
        }

        for (int i = 0; i < m_params.binCount; ++i) m_profile[i] = smoothed[i];

        delete[] smoothed;
    }

    ++m_iterationCount;
}

double control::IterativeLearningControl::correction(double phase) const {
    if (m_profile == nullptr) return 0.0;

    return m_profile[binOf(phase)];
}

double control::IterativeLearningControl::getBin(int i) const {
    if (m_profile == nullptr) return 0.0;
    assert(i >= 0 && i < m_params.binCount);

    return m_profile[i];
}
