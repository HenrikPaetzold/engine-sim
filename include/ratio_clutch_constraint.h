#ifndef ATG_ENGINE_SIM_RATIO_CLUTCH_CONSTRAINT_H
#define ATG_ENGINE_SIM_RATIO_CLUTCH_CONSTRAINT_H

#include "scs.h"

class RatioClutchConstraint : public atg_scs::Constraint {
public:
    RatioClutchConstraint();
    virtual ~RatioClutchConstraint();

    void setInput(atg_scs::RigidBody *body) { m_bodies[0] = body; }
    void setOutput(atg_scs::RigidBody *body) { m_bodies[1] = body; }

    virtual void calculate(Output *output, atg_scs::SystemState *system);

    double getTorque() const { return F_t[0][0]; }
    double getSlipSpeed() const;
    double getSlipPower() const;

    double m_ratio;
    double m_capacity;
    double m_pressure;

    double m_ks;
    double m_kd;
};

#endif /* ATG_ENGINE_SIM_RATIO_CLUTCH_CONSTRAINT_H */
