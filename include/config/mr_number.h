#ifndef ATG_ENGINE_SIM_MR_NUMBER_H
#define ATG_ENGINE_SIM_MR_NUMBER_H

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace config {

    inline std::string mrNumber(double value) {
        if (!std::isfinite(value)) return "0.0";

        std::ostringstream out;
        out << std::fixed << std::setprecision(12) << value;

        std::string text = out.str();

        const std::size_t dot = text.find('.');
        if (dot == std::string::npos) return text + ".0";

        std::size_t last = text.size();
        while (last > dot + 2 && text[last - 1] == '0') --last;
        text.resize(last);

        return text;
    }

}

#endif /* ATG_ENGINE_SIM_MR_NUMBER_H */
