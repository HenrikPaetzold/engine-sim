#include "../include/vehicle_drag_constraint.h"

#include "../include/vehicle.h"

VehicleDragConstraint::VehicleDragConstraint() : Constraint(1, 1) {
    m_ks = 10.0;
    m_kd = 1.0;

    m_vehicle = nullptr;
}

VehicleDragConstraint::~VehicleDragConstraint() {
    /* void */
}

void VehicleDragConstraint::initialize(atg_scs::RigidBody *rotatingMass, Vehicle *vehicle) {
    m_bodies[0] = rotatingMass;
    m_vehicle = vehicle;
}

void VehicleDragConstraint::calculate(Output *output, atg_scs::SystemState *system) {
    output->C[0] = 0;

    output->J[0][0] = 0.0;
    output->J[0][1] = 0.0;
    output->J[0][2] = -1.0;

    output->J[0][3] = 0.0;
    output->J[0][4] = 0.0;
    output->J[0][5] = 1.0;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->J_dot[0][3] = 0;
    output->J_dot[0][4] = 0;
    output->J_dot[0][5] = 0;

    output->kd[0] = m_kd;
    output->ks[0] = m_ks;

    output->v_bias[0] = 0;

    const double dissipative =
        m_vehicle->linearForceToVirtualTorque(
            m_vehicle->getRollingDragForce() + m_vehicle->getAeroDragForce());
    const double grade =
        m_vehicle->linearForceToVirtualTorque(m_vehicle->getGradeForce());

    const double direction = m_vehicle->getTravelDirection();

    double lo, hi;
    if (direction > 0.0) {
        lo = 0.0;
        hi = dissipative;
    }
    else if (direction < 0.0) {
        lo = -dissipative;
        hi = 0.0;
    }
    else {
        lo = -dissipative;
        hi = dissipative;
    }

    lo += grade;
    hi += grade;

    output->limits[0][0] = -hi;
    output->limits[0][1] = -lo;
}
