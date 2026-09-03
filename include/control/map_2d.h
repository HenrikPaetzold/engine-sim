#ifndef ATG_ENGINE_SIM_MAP_2D_H
#define ATG_ENGINE_SIM_MAP_2D_H

namespace control {

    class Map2d {
        public:
            Map2d();
            ~Map2d();

            void initialize(int xCount, int yCount, double initialValue = 0.0);
            void destroy();

            void setXAxis(int i, double x);
            void setYAxis(int j, double y);
            void setValue(int i, int j, double value);
            void fill(double value);

            double getXAxis(int i) const;
            double getYAxis(int j) const;
            double getValue(int i, int j) const;

            double sample(double x, double y) const;

            void locate(
                double x,
                double y,
                int *i0,
                int *j0,
                double *tx,
                double *ty) const;

            void accumulate(double x, double y, double delta, double limitMin, double limitMax);

            inline int getXCount() const { return m_xCount; }
            inline int getYCount() const { return m_yCount; }
            inline bool isInitialized() const { return m_values != nullptr; }

        protected:
            inline int index(int i, int j) const { return j * m_xCount + i; }

            static int findInterval(const double *axis, int count, double v, double *t);

            double *m_xAxis;
            double *m_yAxis;
            double *m_values;

            int m_xCount;
            int m_yCount;
    };

} /* namespace control */

#endif /* ATG_ENGINE_SIM_MAP_2D_H */
