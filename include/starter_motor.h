#ifndef ATG_ENGINE_SIM_STARTER_MOTOR_H
#define ATG_ENGINE_SIM_STARTER_MOTOR_H

#include "scs.h"

#include "crankshaft.h"
#include "control/map_2d.h"

class StarterMotor : public atg_scs::Constraint {
public:
    StarterMotor();
    virtual ~StarterMotor();

    void connectCrankshaft(Crankshaft *crankshaft);
    virtual void calculate(Output *output, atg_scs::SystemState *state);

    double availableTorque(double speed) const;
    double targetSpeed() const;

    double m_ks;
    double m_kd;
    double m_maxTorque;
    double m_rotationSpeed;
    double m_temperature;
    bool m_enabled;

    control::Map2d m_torqueMap;
    control::Map2d m_speedMap;
};

#endif /* ATG_ENGINE_SIM_STARTER_MOTOR_H */
