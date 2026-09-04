#include <gtest/gtest.h>

#include "../include/control/control_program.h"
#include "../include/config/parameter_registry.h"

#include "../include/config/shift_recorder.h"

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
