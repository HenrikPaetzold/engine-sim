#ifndef ATG_ENGINE_SIM_JSON_WRITER_H
#define ATG_ENGINE_SIM_JSON_WRITER_H

#include <ostream>
#include <string>

namespace config {

    inline void writeJsonString(std::ostream &out, const std::string &value) {
        out << '"';
        for (char c : value) {
            if (c == '"' || c == '\\') out << '\\' << c;
            else if (c == '\n') out << "\\n";
            else out << c;
        }
        out << '"';
    }

    inline std::string jsonString(const std::string &value) {
        std::string out = "\"";
        for (char c : value) {
            if (c == '"' || c == '\\') { out += '\\'; out += c; }
            else if (c == '\n') out += "\\n";
            else out += c;
        }

        return out + "\"";
    }

} /* namespace config */

#endif /* ATG_ENGINE_SIM_JSON_WRITER_H */
