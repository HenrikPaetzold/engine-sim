#include "../../include/control/map_2d.h"

#include <algorithm>
#include <cassert>

control::Map2d::Map2d() {
    m_xAxis = nullptr;
    m_yAxis = nullptr;
    m_values = nullptr;
    m_xCount = 0;
    m_yCount = 0;
}

control::Map2d::~Map2d() {
    destroy();
}

void control::Map2d::initialize(int xCount, int yCount, double initialValue) {
    assert(xCount > 0 && yCount > 0);

    destroy();

    m_xCount = xCount;
    m_yCount = yCount;
    m_xAxis = new double[xCount];
    m_yAxis = new double[yCount];
    m_values = new double[xCount * yCount];

    for (int i = 0; i < xCount; ++i) m_xAxis[i] = static_cast<double>(i);
    for (int j = 0; j < yCount; ++j) m_yAxis[j] = static_cast<double>(j);

    fill(initialValue);
}

void control::Map2d::destroy() {
    delete[] m_xAxis;
    delete[] m_yAxis;
    delete[] m_values;

    m_xAxis = nullptr;
    m_yAxis = nullptr;
    m_values = nullptr;
    m_xCount = 0;
    m_yCount = 0;
}

void control::Map2d::setXAxis(int i, double x) {
    assert(i >= 0 && i < m_xCount);
    m_xAxis[i] = x;
}

void control::Map2d::setYAxis(int j, double y) {
    assert(j >= 0 && j < m_yCount);
    m_yAxis[j] = y;
}

void control::Map2d::setValue(int i, int j, double value) {
    assert(i >= 0 && i < m_xCount && j >= 0 && j < m_yCount);
    m_values[index(i, j)] = value;
}

void control::Map2d::fill(double value) {
    for (int i = 0; i < m_xCount * m_yCount; ++i) m_values[i] = value;
}

double control::Map2d::getXAxis(int i) const {
    assert(i >= 0 && i < m_xCount);
    return m_xAxis[i];
}

double control::Map2d::getYAxis(int j) const {
    assert(j >= 0 && j < m_yCount);
    return m_yAxis[j];
}

double control::Map2d::getValue(int i, int j) const {
    assert(i >= 0 && i < m_xCount && j >= 0 && j < m_yCount);
    return m_values[index(i, j)];
}

int control::Map2d::findInterval(const double *axis, int count, double v, double *t) {
    if (count == 1) {
        *t = 0.0;
        return 0;
    }

    if (v <= axis[0]) {
        *t = 0.0;
        return 0;
    }

    if (v >= axis[count - 1]) {
        *t = 1.0;
        return count - 2;
    }

    int lo = 0;
    int hi = count - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (axis[mid] <= v) lo = mid;
        else hi = mid;
    }

    const double span = axis[lo + 1] - axis[lo];
    *t = (span > 0.0) ? (v - axis[lo]) / span : 0.0;

    return lo;
}

void control::Map2d::locate(
    double x,
    double y,
    int *i0,
    int *j0,
    double *tx,
    double *ty) const
{
    *i0 = findInterval(m_xAxis, m_xCount, x, tx);
    *j0 = findInterval(m_yAxis, m_yCount, y, ty);
}

double control::Map2d::sample(double x, double y) const {
    if (m_values == nullptr) return 0.0;

    int i0, j0;
    double tx, ty;
    locate(x, y, &i0, &j0, &tx, &ty);

    const int i1 = (m_xCount > 1) ? i0 + 1 : i0;
    const int j1 = (m_yCount > 1) ? j0 + 1 : j0;

    const double v00 = m_values[index(i0, j0)];
    const double v10 = m_values[index(i1, j0)];
    const double v01 = m_values[index(i0, j1)];
    const double v11 = m_values[index(i1, j1)];

    const double v0 = v00 + (v10 - v00) * tx;
    const double v1 = v01 + (v11 - v01) * tx;

    return v0 + (v1 - v0) * ty;
}

void control::Map2d::accumulate(
    double x,
    double y,
    double delta,
    double limitMin,
    double limitMax)
{
    if (m_values == nullptr) return;

    int i0, j0;
    double tx, ty;
    locate(x, y, &i0, &j0, &tx, &ty);

    const int i1 = (m_xCount > 1) ? i0 + 1 : i0;
    const int j1 = (m_yCount > 1) ? j0 + 1 : j0;

    const double w00 = (1 - tx) * (1 - ty);
    const double w10 = tx * (1 - ty);
    const double w01 = (1 - tx) * ty;
    const double w11 = tx * ty;

    const int indices[4] = {
        index(i0, j0), index(i1, j0), index(i0, j1), index(i1, j1) };
    const double weights[4] = { w00, w10, w01, w11 };

    for (int k = 0; k < 4; ++k) {
        if (weights[k] == 0.0) continue;

        m_values[indices[k]] = std::clamp(
            m_values[indices[k]] + weights[k] * delta,
            limitMin,
            limitMax);
    }
}
