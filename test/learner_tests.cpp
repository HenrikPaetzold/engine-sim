#include <gtest/gtest.h>

#include "../include/control/control_program.h"
#include "../include/config/parameter_registry.h"

#include "../include/config/shift_recorder.h"
#include "../include/powertrain/transmission_control_unit.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace {
    struct Rig {
        control::ControlProgram program;
        config::ParameterRegistry registry;

        double target = 0.0;
        double plant = 0.0;

        control::ConstantBlock *error = nullptr;
        control::ConstantBlock *enable = nullptr;
        control::LearnerBlock *learner = nullptr;

        void build(bool adaptive, double rate = 1.0) {
            config::ParameterDescriptor d =
                config::describeScalar("program.gain", -100.0, 100.0, 0.0, "");
            d.adaptive = adaptive;
            d.adaptMin = -2.0;
            d.adaptMax = 2.0;
            registry.registerScalar(d, &target);

            error = new control::ConstantBlock;
            program.addBlock(error);

            enable = new control::ConstantBlock;
            enable->m_value = 1.0;
            program.addBlock(enable);

            learner = new control::LearnerBlock;
            learner->m_name = "learner";
            learner->m_target = "program.gain";
            learner->m_rate = rate;
            program.addBlock(learner);
            learner->addOperand(error->m_index);
            learner->addOperand(enable->m_index);

            program.setRegistry(&registry);
            program.compile();
        }

        void run(double reference, int steps, double dt = 1e-3) {
            for (int i = 0; i < steps; ++i) {
                error->m_value = target - reference;
                program.update(dt);
            }
        }
    };
}

TEST(LearnerTests, ItConvergesOnTheReferenceValue) {
    Rig rig;
    rig.build(true, 5.0);

    rig.run(1.5, 4000);

    EXPECT_NEAR(rig.target, 1.5, 1e-3);
}

TEST(LearnerTests, TheErrorNormFallsMonotonically) {
    Rig rig;
    rig.build(true, 5.0);

    double previous = 1e9;
    for (int block = 0; block < 10; ++block) {
        rig.run(1.0, 200);

        const double error = std::abs(rig.target - 1.0);
        EXPECT_LE(error, previous + 1e-9) << "block " << block;
        previous = error;
    }

    EXPECT_LT(previous, 0.1);
}

TEST(LearnerTests, ItRespectsTheAdaptionLimits) {
    Rig rig;
    rig.build(true, 20.0);

    rig.run(9.0, 8000);

    EXPECT_NEAR(rig.target, 2.0, 1e-9) << "learner exceeded adaptMax";

    rig.run(-9.0, 8000);
    EXPECT_NEAR(rig.target, -2.0, 1e-9) << "learner exceeded adaptMin";
}

TEST(LearnerTests, ANonAdaptiveParameterIsNeverTouched) {
    Rig rig;
    rig.build(false, 5.0);

    rig.run(1.5, 4000);

    EXPECT_NEAR(rig.target, 0.0, 1e-12) << "learner wrote a fixed parameter";
}

TEST(LearnerTests, DisablingFreezesTheValue) {
    Rig rig;
    rig.build(true, 5.0);

    rig.run(1.0, 400);
    const double frozen = rig.target;
    ASSERT_GT(frozen, 0.0);

    rig.enable->m_value = 0.0;
    rig.run(1.0, 4000);

    EXPECT_NEAR(rig.target, frozen, 1e-12);
}

TEST(LearnerTests, ItReportsTheTargetValueAsItsOutput) {
    Rig rig;
    rig.build(true, 5.0);

    rig.run(1.0, 1000);

    EXPECT_NEAR(rig.learner->m_output, rig.target, 1e-12);
}

TEST(LearnerTests, WithoutARegistryItIsAHarmlessNoOp) {
    control::ControlProgram program;

    control::ConstantBlock *error = new control::ConstantBlock;
    error->m_value = 1.0;
    program.addBlock(error);

    control::LearnerBlock *learner = new control::LearnerBlock;
    learner->m_target = "program.gain";
    learner->m_rate = 5.0;
    program.addBlock(learner);
    learner->addOperand(error->m_index);

    ASSERT_TRUE(program.compile());
    for (int i = 0; i < 100; ++i) program.update(1e-3);

    EXPECT_NEAR(learner->m_output, 0.0, 1e-12);
}

namespace {
    config::ShiftRecorder::Sample shiftSample(double clutch, double slip) {
        config::ShiftRecorder::Sample sample;
        sample.clutchPressure = clutch;
        sample.engineSpeed = 300.0;
        sample.torqueRequest = 120.0;
        sample.torqueReduction = 0.5;
        sample.clutchSlip = slip;

        return sample;
    }

    void driveShift(config::ShiftRecorder &recorder, int gear, double duration) {
        const double dt = 1e-3;
        const int steps = static_cast<int>(duration / dt);

        for (int i = 0; i < steps; ++i) {
            recorder.update(dt, true, gear, shiftSample(0.5, 20.0));
        }

        for (int i = 0; i < 1700; ++i) {
            recorder.update(dt, false, gear, shiftSample(1.0, 0.0));
        }
    }
}

TEST(ShiftRecorderTests, ARisingEdgeStartsExactlyOneRecording) {
    config::ShiftRecorder recorder;
    recorder.initialize(1.5);

    driveShift(recorder, 1, 0.4);

    EXPECT_EQ(recorder.getCount(), 1);
    EXPECT_GT(recorder.get(0).samples.size(), 10u);
}

TEST(ShiftRecorderTests, TwoShiftsGiveTwoSeparateRecordings) {
    config::ShiftRecorder recorder;
    recorder.initialize(1.5);

    driveShift(recorder, 1, 0.4);
    driveShift(recorder, 2, 0.4);

    EXPECT_EQ(recorder.getCount(), 2);
}

TEST(ShiftRecorderTests, ItNeverExceedsItsSampleBudget) {
    config::ShiftRecorder recorder;
    recorder.initialize(1.5);

    driveShift(recorder, 1, 5.0);

    ASSERT_EQ(recorder.getCount(), 1);
    EXPECT_LE(
        static_cast<int>(recorder.get(0).samples.size()),
        config::ShiftRecorder::MaxSamples);
}

TEST(ShiftRecorderTests, ItKeepsOnlyTheMostRecentRecordings) {
    config::ShiftRecorder recorder;
    recorder.initialize(1.5);

    for (int i = 0; i < config::ShiftRecorder::MaxRecordings + 4; ++i) {
        driveShift(recorder, i, 0.3);
    }

    EXPECT_EQ(recorder.getCount(), config::ShiftRecorder::MaxRecordings);
    EXPECT_EQ(
        recorder.get(recorder.getCount() - 1).fromGear,
        config::ShiftRecorder::MaxRecordings + 3);
}

TEST(ShiftRecorderTests, ItSerializesValidJsonWhenEmpty) {
    config::ShiftRecorder recorder;
    recorder.initialize(1.5);

    std::ostringstream out;
    recorder.serializeJson(out);

    EXPECT_EQ(out.str(), "[]");
}

TEST(ShiftRecorderTests, ASerializedRecordingCarriesSixChannels) {
    config::ShiftRecorder recorder;
    recorder.initialize(1.5);
    driveShift(recorder, 1, 0.3);

    std::ostringstream out;
    recorder.serializeJson(out);
    const std::string json = out.str();

    EXPECT_NE(json.find("\"fromGear\""), std::string::npos);
    EXPECT_NE(json.find("\"samples\""), std::string::npos);

    const size_t open = json.find("[[");
    ASSERT_NE(open, std::string::npos);
    const size_t close = json.find(']', open + 2);
    ASSERT_NE(close, std::string::npos);

    const std::string first = json.substr(open + 2, close - open - 2);
    EXPECT_EQ(std::count(first.begin(), first.end(), ','), 5);
}

namespace {
    struct ZoneRig {
        control::ControlProgram program;
        config::ParameterRegistry registry;
        control::Map2d map;

        control::ConstantBlock *error = nullptr;
        control::ConstantBlock *enable = nullptr;
        control::ConstantBlock *x = nullptr;
        control::ConstantBlock *y = nullptr;
        control::LearnerBlock *learner = nullptr;

        void build(bool adaptive, double rate = 1.0) {
            map.initialize(3, 2, 0.0);
            map.setXAxis(0, 0.0);
            map.setXAxis(1, 50.0);
            map.setXAxis(2, 100.0);
            map.setYAxis(0, 0.0);
            map.setYAxis(1, 10.0);

            config::ParameterDescriptor d =
                config::describeScalar("tcu.zone_map", -5.0, 5.0, 0.0, "");
            d.type = config::ParameterType::Map;
            d.adaptive = adaptive;
            d.adaptMin = -5.0;
            d.adaptMax = 5.0;
            registry.registerMap(d, &map);

            error = new control::ConstantBlock;
            program.addBlock(error);

            enable = new control::ConstantBlock;
            enable->m_value = 1.0;
            program.addBlock(enable);

            x = new control::ConstantBlock;
            program.addBlock(x);

            y = new control::ConstantBlock;
            program.addBlock(y);

            learner = new control::LearnerBlock;
            learner->m_name = "learner";
            learner->m_target = "tcu.zone_map";
            learner->m_rate = rate;
            program.addBlock(learner);
            learner->addOperand(error->m_index);
            learner->addOperand(enable->m_index);
            learner->addOperand(x->m_index);
            learner->addOperand(y->m_index);

            program.setRegistry(&registry);
            program.compile();
        }

        void run(double at, double errorValue, int steps, double dt = 1e-3) {
            x->m_value = at;
            error->m_value = errorValue;
            for (int i = 0; i < steps; ++i) program.update(dt);
        }
    };
}

TEST(ZoneLearnerTests, AMarkedMapLearnsAtTheOperatingPoint) {
    ZoneRig rig;
    rig.build(true);

    rig.run(50.0, -1.0, 1000);

    EXPECT_GT(rig.map.getValue(1, 0), 0.0);
    EXPECT_NEAR(rig.map.getValue(0, 0), 0.0, 1e-12);
    EXPECT_NEAR(rig.map.getValue(2, 0), 0.0, 1e-12);
}

TEST(ZoneLearnerTests, WithoutTheAdaptiveFlagNothingIsLearned) {
    ZoneRig rig;
    rig.build(false);

    rig.run(50.0, -1.0, 1000);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(rig.map.getValue(i, 0), 0.0, 1e-12);
    }
}

TEST(ZoneLearnerTests, SetAdaptiveOpensTheGate) {
    ZoneRig rig;
    rig.build(false);

    rig.run(50.0, -1.0, 500);
    ASSERT_NEAR(rig.map.getValue(1, 0), 0.0, 1e-12);

    ASSERT_TRUE(rig.registry.setAdaptive("tcu.zone_map", true, -5.0, 5.0));
    rig.run(50.0, -1.0, 500);

    EXPECT_GT(rig.map.getValue(1, 0), 0.0);
}

TEST(ZoneLearnerTests, TheLearnedZoneIsBounded) {
    ZoneRig rig;
    rig.build(true, 50.0);

    rig.run(50.0, -1.0, 20000);

    EXPECT_NEAR(rig.map.getValue(1, 0), 5.0, 1e-9);
}

TEST(ZoneLearnerTests, TwoZonesLearnIndependently) {
    ZoneRig rig;
    rig.build(true);

    rig.run(0.0, -1.0, 1000);
    rig.run(100.0, 2.0, 1000);

    EXPECT_GT(rig.map.getValue(0, 0), 0.0);
    EXPECT_LT(rig.map.getValue(2, 0), 0.0);
    EXPECT_NEAR(rig.map.getValue(1, 0), 0.0, 1e-12);
}

TEST(RegistryAdaptTests, ACellPathIsAdaptableWhenTheMapIsMarked) {
    config::ParameterRegistry registry;
    control::Map2d map;
    map.initialize(2, 2, 0.0);
    map.setXAxis(0, 0.0);
    map.setXAxis(1, 1.0);
    map.setYAxis(0, 0.0);
    map.setYAxis(1, 1.0);

    config::ParameterDescriptor d =
        config::describeScalar("tcu.cells", -1.0, 1.0, 0.0, "");
    d.type = config::ParameterType::Map;
    d.adaptive = true;
    d.adaptMin = -0.5;
    d.adaptMax = 0.5;
    registry.registerMap(d, &map);

    EXPECT_TRUE(registry.adapt("tcu.cells[1][1]", 0.2));
    EXPECT_NEAR(map.getValue(1, 1), 0.2, 1e-12);

    EXPECT_TRUE(registry.adapt("tcu.cells[1][1]", 10.0));
    EXPECT_NEAR(map.getValue(1, 1), 0.5, 1e-12);

    EXPECT_FALSE(registry.adapt("tcu.cells[9][0]", 0.1));
    EXPECT_FALSE(registry.adapt("tcu.cells", 0.1));
}

TEST(RegistryAdaptTests, AnUnmarkedMapRefusesTheCellPath) {
    config::ParameterRegistry registry;
    control::Map2d map;
    map.initialize(2, 2, 0.0);
    map.setXAxis(0, 0.0);
    map.setXAxis(1, 1.0);
    map.setYAxis(0, 0.0);
    map.setYAxis(1, 1.0);

    config::ParameterDescriptor d =
        config::describeScalar("tcu.cells", -1.0, 1.0, 0.0, "");
    d.type = config::ParameterType::Map;
    registry.registerMap(d, &map);

    EXPECT_FALSE(registry.adapt("tcu.cells[1][1]", 0.2));
    EXPECT_NEAR(map.getValue(1, 1), 0.0, 1e-12);
}

TEST(RegistryAdaptTests, TheTcuScheduleMapsAreOpenForLearning) {
    config::ParameterRegistry registry;
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(powertrain::TransmissionControlUnit::Parameters());
    tcu.registerParameters(&registry);

    EXPECT_TRUE(registry.isAdaptive("tcu.upshift_map"));
    EXPECT_TRUE(registry.isAdaptive("tcu.downshift_map"));
    EXPECT_TRUE(registry.isAdaptive("tcu.lockup_map"));
}
