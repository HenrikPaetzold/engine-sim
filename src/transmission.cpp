#include "../include/transmission.h"

#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <algorithm>
#include <cmath>

Transmission::Transmission() {
    m_type = Type::Legacy;
    m_engagement = powertrain::GateEngagement::Forward;
    m_gear = -1;
    m_preselectedGear = -1;
    m_gearCount = 0;
    m_gearRatios = nullptr;
    m_maxClutchTorque = units::torque(1000.0, units::ft_lb);
    m_rotatingMass = nullptr;
    m_vehicle = nullptr;
    m_engine = nullptr;
    m_lockupPressure = 0.0;
    m_turbineInertia = 0.08;
    m_reverseRatio = 3.2;
    m_parkLockTorque = units::torque(4000.0, units::Nm);

    for (int i = 0; i < ClutchCount; ++i) {
        m_clutchPressure[i] = 0.0;
        m_clutchGear[i] = -1;
    }
}

Transmission::~Transmission() {
    if (m_gearRatios != nullptr) {
        delete[] m_gearRatios;
    }

    m_gearRatios = nullptr;
}

void Transmission::initialize(const Parameters &params) {
    m_gearCount = params.GearCount;
    m_maxClutchTorque = params.MaxClutchTorque;
    m_gearRatios = new double[params.GearCount];
    memcpy(m_gearRatios, params.GearRatios, sizeof(double) * m_gearCount);

    m_type = params.GearboxType;
    m_turbineInertia = params.TurbineInertia;
    m_reverseRatio = params.ReverseRatio;
    m_parkLockTorque = params.ParkLockTorque;

    m_converter.m_stallTorqueRatio = params.StallTorqueRatio;
    m_converter.m_couplingPoint = params.CouplingPoint;
    m_converter.m_capacityFactor = params.CapacityFactor;
}

void Transmission::registerParameters(config::ParameterRegistry *registry, const char *prefix) {
    if (registry == nullptr) return;

    const std::string base = std::string(prefix) + "driveline.";

    registry->registerScalar(
        config::describeScalar(base + "clutch_torque", 0.0, units::torque(5000.0, units::ft_lb),
            m_maxClutchTorque, "Nm"),
        &m_maxClutchTorque);
    registry->registerScalar(
        config::describeScalar(base + "turbine_inertia", 1e-3, 5.0, m_turbineInertia, "kgm2"),
        &m_turbineInertia);
    registry->registerScalar(
        config::describeScalar(base + "reverse_ratio", 0.5, 12.0, m_reverseRatio, ""),
        &m_reverseRatio);
    registry->registerScalar(
        config::describeScalar(base + "park_lock_torque", 0.0,
            units::torque(50000.0, units::Nm), m_parkLockTorque, "Nm"),
        &m_parkLockTorque);
    registry->registerScalar(
        config::describeScalar(base + "converter.stall_torque_ratio", 1.0, 4.0,
            m_converter.m_stallTorqueRatio, ""),
        &m_converter.m_stallTorqueRatio);
    registry->registerScalar(
        config::describeScalar(base + "converter.coupling_point", 0.1, 1.0,
            m_converter.m_couplingPoint, ""),
        &m_converter.m_couplingPoint);
    registry->registerScalar(
        config::describeScalar(base + "converter.capacity_factor", 1e-5, 1.0,
            m_converter.m_capacityFactor, "Nm/(rad/s)2"),
        &m_converter.m_capacityFactor);
}

bool Transmission::requiresTorqueInterrupt() const {
    return m_type != Type::DualClutch && m_type != Type::Converter;
}

void Transmission::setClutchPressure(int index, double pressure) {
    if (index < 0 || index >= ClutchCount) return;
    m_clutchPressure[index] = pressure;
}

double Transmission::getClutchPressure(int index) const {
    if (index < 0 || index >= ClutchCount) return 0.0;
    return m_clutchPressure[index];
}

void Transmission::setEngagement(powertrain::GateEngagement range) {
    if (!supportsEngagement()
        && (range == powertrain::GateEngagement::Park
            || range == powertrain::GateEngagement::Reverse))
    {
        m_engagement = powertrain::GateEngagement::Neutral;
        return;
    }

    m_engagement = range;
}

double Transmission::getParkLockTorque() const {
    return m_parkLockTorque;
}

void Transmission::addParkLockForTest(atg_scs::RigidBodySystem *system) {
    m_parkLock.setBody(m_rotatingMass);
    m_parkLock.m_minTorque = 0.0;
    m_parkLock.m_maxTorque = 0.0;
    system->addConstraint(&m_parkLock);
}

void Transmission::setClutchGear(int clutch, int gear) {
    if (clutch < 0 || clutch >= ClutchCount) return;
    if (gear < -1 || gear >= m_gearCount) return;

    m_clutchGear[clutch] = gear;
}

int Transmission::getClutchGear(int clutch) const {
    if (clutch < 0 || clutch >= ClutchCount) return -1;
    return m_clutchGear[clutch];
}

void Transmission::setPreselectedGear(int gear) {
    if (gear < -1 || gear >= m_gearCount) return;
    m_preselectedGear = gear;
}

double Transmission::referenceRatio() const {
    if (m_vehicle == nullptr) return 1.0;

    const double diffRatio = m_vehicle->getDiffRatio();
    if (diffRatio == 0.0) return 1.0;

    return m_vehicle->getTireRadius() / diffRatio;
}

double Transmission::getDrivelineInertia() const {
    if (m_vehicle == nullptr) return 1.0;

    const double f = referenceRatio();
    return m_vehicle->getMass() * f * f;
}

void Transmission::updateDrivelineInertia() {
    if (m_rotatingMass == nullptr) return;

    const double inertia = getDrivelineInertia();
    if (inertia > 0.0) m_rotatingMass->I = inertia;
}

void Transmission::updateRatioClutches() {
    int gears[ClutchCount] = { m_gear, m_preselectedGear };

    bool assigned = false;
    for (int i = 0; i < ClutchCount; ++i) {
        if (m_clutchGear[i] >= 0) assigned = true;
    }

    if (m_type == Type::DualClutch && assigned) {
        for (int i = 0; i < ClutchCount; ++i) gears[i] = m_clutchGear[i];
    }

    for (int i = 0; i < ClutchCount; ++i) {
        RatioClutchConstraint &clutch = m_ratioClutch[i];

        if (i == 0 && m_engagement == powertrain::GateEngagement::Reverse) {
            clutch.m_ratio = -m_reverseRatio;
            clutch.m_capacity = m_maxClutchTorque;
            clutch.m_pressure = m_clutchPressure[i];
        }
        else if (gears[i] < 0 || gears[i] >= m_gearCount
            || m_engagement == powertrain::GateEngagement::Park
            || m_engagement == powertrain::GateEngagement::Neutral)
        {
            clutch.m_ratio = 1.0;
            clutch.m_capacity = 0.0;
            clutch.m_pressure = 0.0;
        }
        else {
            clutch.m_ratio = m_gearRatios[gears[i]];
            clutch.m_capacity = m_maxClutchTorque;
            clutch.m_pressure = m_clutchPressure[i];
        }
    }

    if (m_type != Type::DualClutch) {
        m_ratioClutch[1].m_capacity = 0.0;
        m_ratioClutch[1].m_pressure = 0.0;
    }
}

void Transmission::update(double dt) {
    if (m_type == Type::Legacy) {
        if (m_gear == -1) {
            m_clutchConstraint.m_minTorque = 0;
            m_clutchConstraint.m_maxTorque = 0;
        }
        else {
            m_clutchConstraint.m_minTorque = -m_maxClutchTorque * m_clutchPressure[0];
            m_clutchConstraint.m_maxTorque = m_maxClutchTorque * m_clutchPressure[0];
        }

        return;
    }

    updateDrivelineInertia();
    updateRatioClutches();

    if (m_type == Type::Converter) {
        m_lockupClutch.m_ratio = 1.0;
        m_lockupClutch.m_capacity = m_maxClutchTorque;
        m_lockupClutch.m_pressure = m_lockupPressure;
    }

    if (m_engagement == powertrain::GateEngagement::Park) {
        m_parkLock.m_minTorque = -m_parkLockTorque;
        m_parkLock.m_maxTorque = m_parkLockTorque;
    }
    else {
        m_parkLock.m_minTorque = 0.0;
        m_parkLock.m_maxTorque = 0.0;
    }
}

void Transmission::bind(
    atg_scs::RigidBody *rotatingMass,
    Vehicle *vehicle,
    Engine *engine)
{
    m_rotatingMass = rotatingMass;
    m_vehicle = vehicle;
    m_engine = engine;
}

double Transmission::getClutchRatio(int index) const {
    if (index < 0 || index >= ClutchCount) return 0.0;
    return m_ratioClutch[index].m_ratio;
}

double Transmission::getClutchCapacity(int index) const {
    if (index < 0 || index >= ClutchCount) return 0.0;
    return m_ratioClutch[index].m_capacity * m_ratioClutch[index].m_pressure;
}

double Transmission::getClutchTorque(int index) const {
    if (index < 0 || index >= ClutchCount) return 0.0;
    return m_ratioClutch[index].getTorque();
}

void Transmission::addToSystem(
    atg_scs::RigidBodySystem *system,
    atg_scs::RigidBody *rotatingMass,
    Vehicle *vehicle,
    Engine *engine)
{
    bind(rotatingMass, vehicle, engine);

    atg_scs::RigidBody *crankshaft = &engine->getOutputCrankshaft()->m_body;

    if (m_type == Type::Legacy) {
        m_clutchConstraint.setBody1(crankshaft);
        m_clutchConstraint.setBody2(m_rotatingMass);

        system->addConstraint(&m_clutchConstraint);
        return;
    }

    atg_scs::RigidBody *input = crankshaft;

    if (m_type == Type::Converter) {
        m_turbine.reset();
        m_turbine.m = 1.0;
        m_turbine.I = std::max(m_turbineInertia, 1e-3);
        system->addRigidBody(&m_turbine);

        m_converter.setPump(crankshaft);
        m_converter.setTurbine(&m_turbine);
        system->addConstraint(&m_converter);

        m_lockupClutch.setInput(crankshaft);
        m_lockupClutch.setOutput(&m_turbine);
        m_lockupClutch.m_ratio = 1.0;
        system->addConstraint(&m_lockupClutch);

        input = &m_turbine;
    }

    const int clutches = (m_type == Type::DualClutch) ? ClutchCount : 1;
    for (int i = 0; i < clutches; ++i) {
        m_ratioClutch[i].setInput(input);
        m_ratioClutch[i].setOutput(m_rotatingMass);
        system->addConstraint(&m_ratioClutch[i]);
    }

    m_parkLock.setBody(m_rotatingMass);
    m_parkLock.m_minTorque = 0.0;
    m_parkLock.m_maxTorque = 0.0;
    system->addConstraint(&m_parkLock);
}

double Transmission::getGearRatio(int gear) const {
    if (gear < 0 || gear >= m_gearCount) return 0.0;
    return m_gearRatios[gear];
}

double Transmission::getInputSpeed() const {
    if (m_engine == nullptr) return 0.0;
    return m_engine->getOutputCrankshaft()->m_body.v_theta;
}

double Transmission::getOutputSpeed() const {
    if (m_rotatingMass == nullptr) return 0.0;
    return m_rotatingMass->v_theta;
}

double Transmission::getTurbineSpeed() const {
    if (m_type != Type::Converter) return getInputSpeed();
    return m_turbine.v_theta;
}

double Transmission::getConverterSlip() const {
    if (m_type != Type::Converter) return 0.0;
    return getInputSpeed() - m_turbine.v_theta;
}

double Transmission::getClutchSlipSpeed(int index) const {
    if (m_type == Type::Legacy) return getInputSpeed() - getOutputSpeed();
    if (index < 0 || index >= ClutchCount) return 0.0;

    return m_ratioClutch[index].getSlipSpeed();
}

double Transmission::getClutchSlipSpeed() const {
    return getClutchSlipSpeed(0);
}

void Transmission::changeGear(int newGear) {
    if (newGear < -1 || newGear >= m_gearCount) return;

    if (m_type != Type::Legacy) {
        m_gear = newGear;
        return;
    }

    if (newGear != -1) {
        const double m_car = m_vehicle->getMass();
        const double gear_ratio = m_gearRatios[newGear];
        const double diff_ratio = m_vehicle->getDiffRatio();
        const double tire_radius = m_vehicle->getTireRadius();
        const double f = tire_radius / (diff_ratio * gear_ratio);

        const double new_I = m_car * f * f;
        const double E_r =
            0.5 * m_rotatingMass->I * m_rotatingMass->v_theta * m_rotatingMass->v_theta;
        const double new_v_theta = m_rotatingMass->v_theta < 0
            ? -std::sqrt(E_r * 2 / new_I)
            : std::sqrt(E_r * 2 / new_I);

        m_rotatingMass->I = new_I;
        m_rotatingMass->p_x = m_rotatingMass->p_y = 0;
        m_rotatingMass->m = m_car;
        m_rotatingMass->v_theta = new_v_theta;
    }

    m_gear = newGear;
}
