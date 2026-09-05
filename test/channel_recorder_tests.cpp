#include <gtest/gtest.h>

#include "../include/config/channel_recorder.h"
#include "../include/powertrain/powertrain_unit.h"
#include "../include/units.h"

#include <sstream>

namespace {
    config::ChannelTable buildTable() {
        config::ChannelTable table;
        table.set("engine_rpm", 1000.0);
        table.set("clutch_pressure", 0.5);
        table.set("vehicle_speed", 20.0);

        return table;
    }
}

TEST(ChannelTableTests, ANameIsDefinedOnceAndKeepsItsIndex) {
    config::ChannelTable table;

    const int first = table.define("engine_rpm");
    const int again = table.define("engine_rpm");

    EXPECT_EQ(first, again);
    EXPECT_EQ(table.getCount(), 1);
    EXPECT_EQ(table.find("engine_rpm"), first);
    EXPECT_EQ(table.find("missing"), -1);

    table.set(first, 4200.0);
    EXPECT_NEAR(table.getValue(first), 4200.0, 1e-12);
    EXPECT_NEAR(table.getValue(-1), 0.0, 1e-12);
}

TEST(ChannelRecorderTests, ItRecordsOnlyTheSelectedChannels) {
    config::ChannelTable table = buildTable();

    config::ChannelRecorder recorder;
    recorder.initialize(1.0);
    recorder.select({ "engine_rpm", "vehicle_speed" });

    for (int i = 0; i < 1000; ++i) recorder.update(1e-3, i * 1e-3, table);

    ASSERT_EQ(recorder.getChannelCount(), 2);
    EXPECT_EQ(recorder.getChannelName(0), "engine_rpm");
    EXPECT_EQ(recorder.getChannelName(1), "vehicle_speed");
    EXPECT_GT(recorder.getSampleCount(), 0);
    EXPECT_NEAR(recorder.getSample(0, 0), 1000.0, 1e-9);
    EXPECT_NEAR(recorder.getSample(1, 0), 20.0, 1e-9);
}

TEST(ChannelRecorderTests, AShortEventKeepsItsShape) {
    config::ChannelTable table;
    const int pressure = table.define("clutch_pressure");

    config::ChannelRecorder recorder;
    recorder.initialize(1.0);
    recorder.select({ "clutch_pressure" });

    int inside = 0;
    for (int i = 0; i < 1000; ++i) {
        const double t = i * 1e-3;
        table.set(pressure, (t >= 0.4 && t < 0.6) ? 0.25 : 1.0);
        recorder.update(1e-3, t, table);
    }

    for (int i = 0; i < recorder.getSampleCount(); ++i) {
        if (recorder.getSample(0, i) < 0.5) ++inside;
    }

    EXPECT_GE(inside, 100)
        << "a 200 ms event must survive with enough detail to read its shape";
}

TEST(ChannelRecorderTests, TheRollingWindowKeepsTheNewestSamples) {
    config::ChannelTable table;
    const int ramp = table.define("ramp");

    config::ChannelRecorder recorder;
    recorder.initialize(1.0);
    recorder.select({ "ramp" });

    for (int i = 0; i < 5000; ++i) {
        table.set(ramp, static_cast<double>(i));
        recorder.update(1e-3, i * 1e-3, table);
    }

    ASSERT_GT(recorder.getSampleCount(), 0);
    EXPECT_LE(recorder.getSampleCount(), config::ChannelRecorder::MaxSamples);

    const double last = recorder.getSample(0, recorder.getSampleCount() - 1);
    EXPECT_GT(last, 4000.0) << "the window did not follow the newest data";
}

TEST(ChannelRecorderTests, ATriggeredRunFillsOnceAndStops) {
    config::ChannelTable table;
    const int ramp = table.define("ramp");

    config::ChannelRecorder recorder;
    recorder.initialize(1.0);
    recorder.select({ "ramp" });
    recorder.setMode(config::ChannelRecorder::Mode::Triggered);

    for (int i = 0; i < 500; ++i) recorder.update(1e-3, i * 1e-3, table);
    EXPECT_EQ(recorder.getSampleCount(), 0) << "it recorded without being armed";

    recorder.arm();
    for (int i = 0; i < 5000; ++i) {
        table.set(ramp, static_cast<double>(i));
        recorder.update(1e-3, i * 1e-3, table);
    }

    EXPECT_EQ(recorder.getSampleCount(), config::ChannelRecorder::MaxSamples);
    EXPECT_FALSE(recorder.isArmed()) << "it kept recording after the window was full";
    EXPECT_LT(recorder.getSample(0, recorder.getSampleCount() - 1), 4000.0)
        << "a triggered run must keep the beginning, not the end";
}

TEST(ChannelRecorderTests, AnUnknownChannelIsReportedNotGuessed) {
    config::ChannelTable table = buildTable();

    config::ChannelRecorder recorder;
    recorder.initialize(1.0);
    recorder.select({ "engine_rpm", "does_not_exist" });

    for (int i = 0; i < 200; ++i) recorder.update(1e-3, i * 1e-3, table);

    std::ostringstream out;
    recorder.serializeJson(out);
    const std::string json = out.str();

    EXPECT_NE(json.find("\"name\":\"engine_rpm\",\"found\":true"), std::string::npos);
    EXPECT_NE(json.find("\"name\":\"does_not_exist\",\"found\":false"), std::string::npos);
}

TEST(ChannelRecorderTests, MoreChannelsThanTheLimitAreRefused) {
    config::ChannelRecorder recorder;
    recorder.initialize(1.0);
    recorder.select({ "a", "b", "c", "d", "e", "f", "g", "h", "i", "j" });

    EXPECT_EQ(recorder.getChannelCount(), config::ChannelRecorder::MaxChannels);
}

TEST(ControllerChannelTests, ThePidInternalsAreVisibleAndHonest) {
    powertrain::PowertrainUnit unit;
    unit.initialize(
        powertrain::EngineControlUnit::Parameters(),
        powertrain::TransmissionControlUnit::Parameters());

    powertrain::PowertrainState state;
    state.coolantTemperature = units::celcius(90.0);
    state.engineRunning = true;
    state.engineSpeed = units::rpm(400.0);
    state.engineRpm = 400.0;
    state.gear = -1;

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    config::ChannelTable table;
    unit.fillChannels(&table);

    ASSERT_GE(table.find("pid.ecu.idle.i"), 0) << "the idle integrator is not exposed";
    ASSERT_GE(table.find("pid.ecu.idle.error"), 0);
    ASSERT_GE(table.find("pid.ecu.idle.saturated"), 0);

    for (int i = 0; i < 3000; ++i) unit.update(1e-3, state, inputs, &commands);

    unit.fillChannels(&table);

    const double integrator = table.getValue(table.find("pid.ecu.idle.i"));
    const double error = table.getValue(table.find("pid.ecu.idle.error"));

    EXPECT_NEAR(
        integrator,
        unit.getEngineControlUnit().getIdleController().getIntegrator(),
        1e-12) << "the channel does not report the real controller";

    EXPECT_GT(error, 0.0)
        << "the engine is below idle, so the error must be positive";
    EXPECT_GT(integrator, 0.0)
        << "a standing error must wind the integrator up";
}

TEST(ControllerChannelTests, TheSaturationFlagFires) {
    powertrain::PowertrainUnit unit;
    unit.initialize(
        powertrain::EngineControlUnit::Parameters(),
        powertrain::TransmissionControlUnit::Parameters());

    powertrain::PowertrainState state;
    state.coolantTemperature = units::celcius(90.0);
    state.engineRunning = true;
    state.engineSpeed = units::rpm(450.0);
    state.engineRpm = 450.0;
    state.gear = -1;

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    for (int i = 0; i < 20000; ++i) unit.update(1e-3, state, inputs, &commands);

    config::ChannelTable table;
    unit.fillChannels(&table);

    EXPECT_NEAR(table.getValue(table.find("pid.ecu.idle.saturated")), 1.0, 1e-12)
        << "a permanently starved controller must report saturation";
}
