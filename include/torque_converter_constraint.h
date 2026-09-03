#ifndef ATG_ENGINE_SIM_TORQUE_CONVERTER_CONSTRAINT_H
#define ATG_ENGINE_SIM_TORQUE_CONVERTER_CONSTRAINT_H

#include "scs.h"

class Function;

class TorqueConverterConstraint : public atg_scs::Constraint {
public:
    TorqueConverterConstraint();
    virtual ~TorqueConverterConstraint();

    void setPump(atg_scs::RigidBody *body) { m_bodies[0] = body; }
    void setTurbine(atg_scs::RigidBody *body) { m_bodies[1] = body; }

    void setCapacityCurve(Function *curve) { m_capacityCurve = curve; }
    void setTorqueRatioCurve(Function *curve) { m_torqueRatioCurve = curve; }

    virtual void calculate(Output *output, atg_scs::SystemState *system);

    double getSpeedRatio() const;
    double capacityFactor(double speedRatio) const;
    double torqueRatio(double speedRatio) const;

    double getPumpTorque() const { return F_t[0][0]; }
    double getTurbineTorque() const { return F_t[0][1]; }

    double m_stallTorqueRatio;
    double m_couplingPoint;
    double m_capacityFactor;

    double m_ks;
    double m_kd;

private:
    Function *m_capacityCurve;
    Function *m_torqueRatioCurve;
};

#endif /* ATG_ENGINE_SIM_TORQUE_CONVERTER_CONSTRAINT_H */
