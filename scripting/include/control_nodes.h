#ifndef ATG_ENGINE_SIM_CONTROL_NODES_H
#define ATG_ENGINE_SIM_CONTROL_NODES_H

#include "object_reference_node.h"

#include "engine_sim.h"

#include <algorithm>
#include <vector>

namespace es_script {

    class PidControllerNode : public ObjectReferenceNode<PidControllerNode> {
    public:
        PidControllerNode() { /* void */ }
        virtual ~PidControllerNode() { /* void */ }

        const control::PidController::Parameters &getParameters() const {
            return m_parameters;
        }

    protected:
        virtual void registerInputs() override {
            addInput("kp", &m_parameters.kp);
            addInput("ki", &m_parameters.ki);
            addInput("kd", &m_parameters.kd);
            addInput("min", &m_parameters.outputMin);
            addInput("max", &m_parameters.outputMax);
            addInput("d_filter_hz", &m_parameters.derivativeCutoff);
            addInput("anti_windup", &m_parameters.trackingGain);
            addInput("integrator_limit", &m_parameters.integratorLimit);

            ObjectReferenceNode<PidControllerNode>::registerInputs();
        }

        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();
        }

        control::PidController::Parameters m_parameters;
    };

    class Map2dNode : public ObjectReferenceNode<Map2dNode> {
    public:
        struct Sample {
            double x, y, value;
        };

    public:
        Map2dNode() { /* void */ }
        virtual ~Map2dNode() { /* void */ }

        void addSample(double x, double y, double value) {
            m_samples.push_back({ x, y, value });
        }

        bool isEmpty() const { return m_samples.empty(); }

        void generate(control::Map2d *map) const {
            if (m_samples.empty()) return;

            std::vector<double> xAxis;
            std::vector<double> yAxis;

            for (const Sample &sample : m_samples) {
                if (std::find(xAxis.begin(), xAxis.end(), sample.x) == xAxis.end()) {
                    xAxis.push_back(sample.x);
                }
                if (std::find(yAxis.begin(), yAxis.end(), sample.y) == yAxis.end()) {
                    yAxis.push_back(sample.y);
                }
            }

            std::sort(xAxis.begin(), xAxis.end());
            std::sort(yAxis.begin(), yAxis.end());

            map->initialize(
                static_cast<int>(xAxis.size()),
                static_cast<int>(yAxis.size()),
                0.0);

            for (size_t i = 0; i < xAxis.size(); ++i) {
                map->setXAxis(static_cast<int>(i), xAxis[i]);
            }
            for (size_t j = 0; j < yAxis.size(); ++j) {
                map->setYAxis(static_cast<int>(j), yAxis[j]);
            }

            for (const Sample &sample : m_samples) {
                const int i = static_cast<int>(
                    std::find(xAxis.begin(), xAxis.end(), sample.x) - xAxis.begin());
                const int j = static_cast<int>(
                    std::find(yAxis.begin(), yAxis.end(), sample.y) - yAxis.begin());

                map->setValue(i, j, sample.value);
            }
        }

    protected:
        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();
        }

        std::vector<Sample> m_samples;
    };

    class DriveModeNode : public ObjectReferenceNode<DriveModeNode> {
    public:
        DriveModeNode() { /* void */ }
        virtual ~DriveModeNode() { /* void */ }

        void set(const std::string &path, double value) {
            m_mode.set(path, value);
        }

        const config::DriveMode &generate() const { return m_mode; }

    protected:
        virtual void registerInputs() override {
            addInput("name", &m_name);

            ObjectReferenceNode<DriveModeNode>::registerInputs();
        }

        virtual void _evaluate() override {
            setOutput(this);
            readAllInputs();

            m_mode.setName(m_name);
        }

        std::string m_name;
        config::DriveMode m_mode;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_CONTROL_NODES_H */
