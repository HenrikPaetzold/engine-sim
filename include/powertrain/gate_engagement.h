#ifndef ATG_ENGINE_SIM_GATE_ENGAGEMENT_H
#define ATG_ENGINE_SIM_GATE_ENGAGEMENT_H

namespace powertrain {

    enum class GateEngagement {
        Park,
        Reverse,
        Neutral,
        Forward
    };

    inline const char *engagementName(GateEngagement engagement) {
        switch (engagement) {
        case GateEngagement::Park: return "park";
        case GateEngagement::Reverse: return "reverse";
        case GateEngagement::Neutral: return "neutral";
        case GateEngagement::Forward: return "forward";
        default: return "neutral";
        }
    }

    inline GateEngagement engagementFromName(
        const char *name,
        GateEngagement fallback = GateEngagement::Neutral)
    {
        if (name == nullptr) return fallback;
        switch (name[0]) {
        case 'p': case 'P': return GateEngagement::Park;
        case 'r': case 'R': return GateEngagement::Reverse;
        case 'n': case 'N': return GateEngagement::Neutral;
        case 'f': case 'F': case 'd': case 'D': return GateEngagement::Forward;
        default: return fallback;
        }
    }

} /* namespace powertrain */

#endif /* ATG_ENGINE_SIM_GATE_ENGAGEMENT_H */
