#ifndef ATG_ENGINE_SIM_SELECTOR_GATE_H
#define ATG_ENGINE_SIM_SELECTOR_GATE_H

#include "gate_engagement.h"

#include <string>
#include <vector>

namespace powertrain {

    struct GatePosition {
        std::string name;
        GateEngagement engagement = GateEngagement::Neutral;

        double maxEntrySpeed = -1.0;
        double maxExitSpeed = -1.0;
        bool requiresBrake = false;

        std::string mode;
    };

    class SelectorGate {
        public:
            SelectorGate();
            ~SelectorGate();

            void clear();
            void add(const GatePosition &position);
            void buildDefault();

            int getCount() const;
            const GatePosition &get(int index) const;
            int find(const std::string &name) const;
            int clampIndex(int index) const;

            inline bool isEmpty() const { return m_positions.empty(); }

        protected:
            std::vector<GatePosition> m_positions;
    };

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_SELECTOR_GATE_H */
