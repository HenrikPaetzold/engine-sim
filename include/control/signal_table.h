#ifndef ATG_ENGINE_SIM_SIGNAL_TABLE_H
#define ATG_ENGINE_SIM_SIGNAL_TABLE_H

#include <string>
#include <vector>

namespace control {

    class SignalTable {
        public:
            SignalTable();
            ~SignalTable();

            int declare(const std::string &name);
            int find(const std::string &name) const;

            void clearValues();
            void set(int index, double value);
            double get(int index) const;

            inline int getCount() const { return static_cast<int>(m_names.size()); }
            const std::string &getName(int index) const;
            inline const double *getValues() const { return m_values.data(); }

        protected:
            std::vector<std::string> m_names;
            std::vector<double> m_values;
    };

} /* namespace control */

#endif /* ATG_ENGINE_SIM_SIGNAL_TABLE_H */
