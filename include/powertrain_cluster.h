#ifndef ATG_ENGINE_SIM_POWERTRAIN_CLUSTER_H
#define ATG_ENGINE_SIM_POWERTRAIN_CLUSTER_H

#include "ui_element.h"

#include "simulator.h"
#include "oscilloscope.h"
#include "config/config_server.h"

#include <string>

class PowertrainCluster : public UiElement {
    public:
        static constexpr int ShiftHistory = 16;

    public:
        PowertrainCluster();
        virtual ~PowertrainCluster();

        virtual void initialize(EngineSimApplication *app);
        virtual void destroy();

        virtual void update(float dt);
        virtual void render();

        void sample();
        void setSimulator(Simulator *simulator) { m_simulator = simulator; }

    protected:
        void renderScope(
            Oscilloscope *scope,
            const Bounds &bounds,
            const std::string &title,
            bool overlay = false);
        void renderStatus(const Bounds &bounds);
        void renderShiftQuality(const Bounds &bounds);

        Simulator *m_simulator;

        Oscilloscope *m_torqueRequestScope;
        Oscilloscope *m_torqueActualScope;
        Oscilloscope *m_throttleScope;
        Oscilloscope *m_pedalScope;
        Oscilloscope *m_clutchScope;
        Oscilloscope *m_slipScope;

        config::TelemetrySample m_sample;

        double m_shiftQuality[ShiftHistory];
        int m_shiftCount;
        int m_lastShiftIteration;
};

#endif /* ATG_ENGINE_SIM_POWERTRAIN_CLUSTER_H */
