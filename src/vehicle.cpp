#include "../include/vehicle.h"

#include "../include/constants.h"
#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <cmath>

Vehicle::Vehicle() {
    m_rotatingMass = nullptr;
    m_mass = 0;
    m_dragCoefficient = 0;
    m_crossSectionArea = 0;
    m_diffRatio = 0;
    m_tireRadius = 0;
    m_travelledDistance = 0;
    m_roadGrade = 0;
    m_rollingResistance = 0;
}

Vehicle::~Vehicle() {
    /* void */
}

void Vehicle::initialize(const Parameters &params) {
    m_mass = params.mass;
    m_dragCoefficient = params.dragCoefficient;
    m_crossSectionArea = params.crossSectionArea;
    m_diffRatio = params.diffRatio;
    m_tireRadius = params.tireRadius;
    m_rollingResistance = params.rollingResistance;
}

void Vehicle::registerParameters(config::ParameterRegistry *registry, const char *prefix) {
    if (registry == nullptr) return;

    const std::string base = std::string(prefix) + "vehicle.";

    registry->registerScalar(
        config::describeScalar(base + "mass", 100.0, 20000.0, m_mass, "kg"),
        &m_mass);
    registry->registerScalar(
        config::describeScalar(base + "drag_coefficient", 0.0, 2.0, m_dragCoefficient, ""),
        &m_dragCoefficient);
    registry->registerScalar(
        config::describeScalar(base + "cross_section_area", 0.1, 20.0, m_crossSectionArea, "m2"),
        &m_crossSectionArea);
    registry->registerScalar(
        config::describeScalar(base + "diff_ratio", 0.5, 12.0, m_diffRatio, ""),
        &m_diffRatio);
    registry->registerScalar(
        config::describeScalar(base + "tire_radius", 0.05, 1.5, m_tireRadius, "m"),
        &m_tireRadius);
    registry->registerScalar(
        config::describeScalar(base + "rolling_resistance", 0.0, 20000.0, m_rollingResistance, "N"),
        &m_rollingResistance);
    registry->registerScalar(
        config::describeScalar(base + "road_grade", -0.30, 0.30, m_roadGrade, "rad"),
        &m_roadGrade);
}

void Vehicle::update(double dt) {
    m_travelledDistance += getSpeed() * dt;
}

void Vehicle::addToSystem(atg_scs::RigidBodySystem *system, atg_scs::RigidBody *rotatingMass) {
    m_rotatingMass = rotatingMass;
}

double Vehicle::getSpeed() const {
    const double E_r = 0.5 * m_rotatingMass->I * m_rotatingMass->v_theta * m_rotatingMass->v_theta;
    const double vehicleSpeed = std::sqrt(2 * E_r / m_mass);

    return vehicleSpeed;

    // E_r = 0.5 * I * v_theta^2
    // E_k = 0.5 * m * v^2
}

double Vehicle::getSignedSpeed() const {
    const double speed = getSpeed();
    return (m_rotatingMass != nullptr && m_rotatingMass->v_theta < 0)
        ? -speed
        : speed;
}

double Vehicle::getGradeForce() const {
    return m_mass * constants::g * std::sin(m_roadGrade);
}

double Vehicle::linearForceToVirtualTorque(double force) const {
    const double rotationToKineticRatio =
        std::sqrt(m_rotatingMass->I / m_mass);
    return rotationToKineticRatio * force;
}
