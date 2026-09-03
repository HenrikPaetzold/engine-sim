#include <gtest/gtest.h>

#include "../scripting/include/compiler.h"

#include "../include/units.h"

#include <fstream>
#include <string>

namespace {
    const char *ScratchScript = "powertrain_script_test.mr";

    bool writeScript(const std::string &body) {
        std::ofstream file(ScratchScript, std::ios::out);
        if (!file.is_open()) return false;

        file
            << "import \"engine_sim.mr\"\n\n"
            << "units units()\n\n"
            << body;
        file.close();

        return true;
    }

    class ScriptFixture : public ::testing::Test {
        protected:
            void SetUp() override {
                delete es_script::Compiler::output()->powertrain;
                *es_script::Compiler::output() = es_script::Compiler::Output();
            }

            void TearDown() override {
                std::remove(ScratchScript);
            }

            bool run(const std::string &body) {
                if (!writeScript(body)) return false;

                es_script::Compiler compiler;
                compiler.initialize();

                const bool compiled = compiler.compile(ScratchScript);
                if (compiled) compiler.execute();

                compiler.destroy();

                return compiled;
            }
    };
}

TEST_F(ScriptFixture, EmptyScriptProducesNoPowertrain) {
    ASSERT_TRUE(run(""));
    EXPECT_EQ(es_script::Compiler::output()->powertrain, nullptr);
}

TEST_F(ScriptFixture, DefaultPowertrainIsBuilt) {
    ASSERT_TRUE(run("set_powertrain()\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);
    EXPECT_GT(unit->getEngineControlUnit().getParameters().revLimit, 0.0);
}

TEST_F(ScriptFixture, EngineControlUnitParametersReachTheObject) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    ecu: engine_control_unit(\n"
        "        rev_limit: 8200 * units.rpm,\n"
        "        reference_torque: 340 * units.Nm,\n"
        "        idle_speed_warm: 950 * units.rpm))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    const powertrain::EngineControlUnit::Parameters &params =
        unit->getEngineControlUnit().getParameters();

    EXPECT_NEAR(params.revLimit, units::rpm(8200.0), 1e-6);
    EXPECT_NEAR(params.referenceTorque, units::torque(340.0, units::Nm), 1e-6);
    EXPECT_NEAR(params.idleSpeedWarm, units::rpm(950.0), 1e-6);
}

TEST_F(ScriptFixture, PidGainsReachTheController) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    ecu: engine_control_unit(\n"
        "        idle_controller: pid_controller(kp: 0.5, ki: 0.25, kd: 0.125, min: 0.0, max: 1.0)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    const control::PidController::Parameters &pid =
        unit->getEngineControlUnit().getParameters().idleController;

    EXPECT_NEAR(pid.kp, 0.5, 1e-12);
    EXPECT_NEAR(pid.ki, 0.25, 1e-12);
    EXPECT_NEAR(pid.kd, 0.125, 1e-12);
}

TEST_F(ScriptFixture, GearRatiosAreCollected) {
    ASSERT_TRUE(run(
        "tcu_box tcu_box()\n"
        "node tcu_box {\n"
        "    output box: transmission_control_unit(final_drive: 4.10);\n"
        "}\n"
        "set_powertrain(tcu: tcu_box().box)\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);
    EXPECT_NEAR(
        unit->getTransmissionControlUnit().getParameters().finalDrive,
        4.10,
        1e-12);
}

TEST_F(ScriptFixture, TransmissionCapabilityFlagsSelectTheGearboxKind) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(\n"
        "        preselect: true,\n"
        "        torque_interrupt: false,\n"
        "        clutch_overlap_time: 0.12 * units.sec))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    const powertrain::TransmissionControlUnit::Parameters &params =
        unit->getTransmissionControlUnit().getParameters();

    EXPECT_TRUE(params.supportsPreselect);
    EXPECT_FALSE(params.requiresTorqueInterrupt);
    EXPECT_NEAR(params.clutchOverlapTime, 0.12, 1e-12);
}

TEST_F(ScriptFixture, AdaptationSettingsReachTheManager) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    adaptation: adaptation(\n"
        "        lambda: false,\n"
        "        throttle_rate: 1.25,\n"
        "        idle_limit: 0.5))\n"));

    const adaptation::AdaptationManager::Parameters &params =
        es_script::Compiler::output()->adaptation;

    EXPECT_FALSE(params.lambdaEnabled);
    EXPECT_NEAR(params.throttleLearningRate, 1.25, 1e-12);
    EXPECT_NEAR(params.idleTrimLimit, 0.5, 1e-12);
}

TEST_F(ScriptFixture, DriveModesAreFreelyNamedAndCollected) {
    ASSERT_TRUE(run(
        "add_drive_mode(drive_mode(name: \"canyon\")\n"
        "    .set(\"ecu.limiter.rev_limit\", 8000 * units.rpm)\n"
        "    .set(\"tcu.shift.clutch_engage_time\", 0.12))\n"
        "add_drive_mode(drive_mode(name: \"glacier\")\n"
        "    .set(\"ecu.limiter.rev_limit\", 4000 * units.rpm))\n"));

    const config::DriveModeSet &modes = es_script::Compiler::output()->driveModes;

    ASSERT_EQ(modes.getCount(), 2);
    EXPECT_EQ(modes.get(0).getName(), "canyon");
    EXPECT_EQ(modes.get(1).getName(), "glacier");
    EXPECT_EQ(modes.get(0).getOverrideCount(), 2);
    EXPECT_EQ(modes.get(0).getOverride(0).path, "ecu.limiter.rev_limit");
}

TEST_F(ScriptFixture, DriveModesDriveTheRegistry) {
    ASSERT_TRUE(run(
        "set_powertrain(ecu: engine_control_unit(rev_limit: 7000 * units.rpm))\n"
        "add_drive_mode(drive_mode(name: \"track\")\n"
        "    .set(\"ecu.limiter.rev_limit\", 9000 * units.rpm))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry, "");

    config::DriveModeSet modes = es_script::Compiler::output()->driveModes;
    ASSERT_TRUE(modes.select("track", &registry));

    EXPECT_NEAR(
        unit->getEngineControlUnit().getParameters().revLimit,
        units::rpm(9000.0),
        1e-6);
}

TEST_F(ScriptFixture, MapSamplesBuildAGrid) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    ecu: engine_control_unit(\n"
        "        pedal_map: map_2d()\n"
        "            .add_map_sample(x: 0.0, value: 0.0)\n"
        "            .add_map_sample(x: 0.5, value: 0.2)\n"
        "            .add_map_sample(x: 1.0, value: 1.0)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    control::Map2d &map = unit->getEngineControlUnit().getPedalMap();

    ASSERT_EQ(map.getXCount(), 3);
    EXPECT_NEAR(map.sample(0.0, 0.0), 0.0, 1e-12);
    EXPECT_NEAR(map.sample(0.5, 0.0), 0.2, 1e-12);
    EXPECT_NEAR(map.sample(1.0, 0.0), 1.0, 1e-12);
    EXPECT_NEAR(map.sample(0.75, 0.0), 0.6, 1e-12);
}

TEST_F(ScriptFixture, TwoVariantsCanBeDefinedAndTheLastOneWins) {
    ASSERT_TRUE(run(
        "node comfort_ecu {\n"
        "    alias output __out: engine_control_unit(rev_limit: 6000 * units.rpm);\n"
        "}\n"
        "node sport_ecu {\n"
        "    alias output __out: engine_control_unit(rev_limit: 8500 * units.rpm);\n"
        "}\n"
        "set_powertrain(ecu: comfort_ecu())\n"
        "set_powertrain(ecu: sport_ecu())\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);
    EXPECT_NEAR(
        unit->getEngineControlUnit().getParameters().revLimit,
        units::rpm(8500.0),
        1e-6);
}
