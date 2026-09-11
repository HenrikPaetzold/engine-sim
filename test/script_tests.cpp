#include <gtest/gtest.h>

#include "../scripting/include/compiler.h"

#include "../include/powertrain/scripted_control_unit.h"
#include "../include/powertrain/manoeuvre.h"
#include "../include/transmission.h"
#include "../include/vehicle.h"
#include "../include/thermal_model.h"
#include "../include/engine.h"
#include "../include/piston_engine_simulator.h"
#include "../include/combustion_chamber.h"
#include "../include/config/parameter_registry.h"
#include "../include/config/mr_number.h"
#include "../include/units.h"

#include <fstream>
#include <vector>
#include <sstream>
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
                delete es_script::Compiler::output()->controlProgram;
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
    unit->registerParameters(&registry);

    config::DriveModeSet modes = es_script::Compiler::output()->driveModes;
    ASSERT_TRUE(modes.select("track", &registry));

    EXPECT_NEAR(
        unit->getEngineControlUnit().getParameters().revLimit,
        units::rpm(9000.0),
        1e-6);
}

TEST_F(ScriptFixture, ScriptChoosesTheStartingDriveMode) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    ecu: engine_control_unit(rev_limit: 7000 * units.rpm),\n"
        "    default_mode: \"comfort\")\n"
        "add_drive_mode(drive_mode(name: \"sport\")\n"
        "    .set(\"ecu.limiter.rev_limit\", 9000 * units.rpm))\n"
        "add_drive_mode(drive_mode(name: \"comfort\")\n"
        "    .set(\"ecu.limiter.rev_limit\", 5500 * units.rpm))\n"));

    EXPECT_EQ(es_script::Compiler::output()->defaultMode, "comfort");

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    config::DriveModeSet modes = es_script::Compiler::output()->driveModes;
    ASSERT_TRUE(modes.select(es_script::Compiler::output()->defaultMode, &registry));

    EXPECT_NEAR(
        unit->getEngineControlUnit().getParameters().revLimit,
        units::rpm(5500.0),
        1e-6);
}

TEST_F(ScriptFixture, NoDefaultModeLeavesTheScriptValues) {
    ASSERT_TRUE(run(
        "set_powertrain(ecu: engine_control_unit(rev_limit: 7000 * units.rpm))\n"));

    EXPECT_TRUE(es_script::Compiler::output()->defaultMode.empty());
    EXPECT_NEAR(
        es_script::Compiler::output()->powertrain
            ->getEngineControlUnit().getParameters().revLimit,
        units::rpm(7000.0),
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

TEST_F(ScriptFixture, TheGearboxTypeReachesTheTransmission) {
    ASSERT_TRUE(run(
        "set_transmission(\n"
        "    transmission(type: \"dct\", max_clutch_torque: 900 * units.lb_ft)\n"
        "        .add_gear(3.60)\n"
        "        .add_gear(2.19)\n"
        "        .add_gear(1.41))\n"));

    Transmission *transmission = es_script::Compiler::output()->transmission;
    ASSERT_NE(transmission, nullptr);

    EXPECT_EQ(transmission->getType(), Transmission::Type::DualClutch);
    EXPECT_TRUE(transmission->supportsPreselect());
    EXPECT_FALSE(transmission->requiresTorqueInterrupt());
    EXPECT_EQ(transmission->getGearCount(), 3);
    EXPECT_NEAR(transmission->getGearRatio(0), 3.60, 1e-9);
}

TEST_F(ScriptFixture, TheConverterCurveParametersReachTheTransmission) {
    ASSERT_TRUE(run(
        "set_transmission(\n"
        "    transmission(\n"
        "        type: \"converter\",\n"
        "        stall_torque_ratio: 2.4,\n"
        "        coupling_point: 0.9,\n"
        "        capacity_factor: 0.006)\n"
        "        .add_gear(2.80)\n"
        "        .add_gear(1.50))\n"));

    Transmission *transmission = es_script::Compiler::output()->transmission;
    ASSERT_NE(transmission, nullptr);

    EXPECT_EQ(transmission->getType(), Transmission::Type::Converter);
    EXPECT_TRUE(transmission->hasLaunchDevice());

    config::ParameterRegistry registry;
    transmission->registerParameters(&registry);

    double value = 0.0;
    ASSERT_TRUE(registry.get("driveline.converter.stall_torque_ratio", &value));
    EXPECT_NEAR(value, 2.4, 1e-9);
    ASSERT_TRUE(registry.get("driveline.converter.coupling_point", &value));
    EXPECT_NEAR(value, 0.9, 1e-9);
    ASSERT_TRUE(registry.get("driveline.converter.capacity_factor", &value));
    EXPECT_NEAR(value, 0.006, 1e-9);
}

TEST_F(ScriptFixture, AnUnknownGearboxTypeFallsBackToTheLegacyModel) {
    ASSERT_TRUE(run(
        "set_transmission(transmission(type: \"hovercraft\").add_gear(1.0))\n"));

    Transmission *transmission = es_script::Compiler::output()->transmission;
    ASSERT_NE(transmission, nullptr);
    EXPECT_EQ(transmission->getType(), Transmission::Type::Legacy);
}

TEST_F(ScriptFixture, TheGearboxLibraryBuildsEveryKind) {
    struct Case {
        const char *node;
        Transmission::Type type;
    };

    const Case cases[] = {
        { "manual_gearbox()", Transmission::Type::Manual },
        { "robotised_manual_gearbox()", Transmission::Type::Manual },
        { "dual_clutch_gearbox()", Transmission::Type::DualClutch },
        { "converter_gearbox()", Transmission::Type::Converter } };

    for (const Case &c : cases) {
        delete es_script::Compiler::output()->powertrain;
        delete es_script::Compiler::output()->controlProgram;
        *es_script::Compiler::output() = es_script::Compiler::Output();

        const std::string body =
            std::string("set_transmission(") + c.node + ".add_gear(3.0).add_gear(1.5))\n";
        ASSERT_TRUE(run(body)) << c.node;

        Transmission *transmission = es_script::Compiler::output()->transmission;
        ASSERT_NE(transmission, nullptr) << c.node;
        EXPECT_EQ(transmission->getType(), c.type) << c.node;
        EXPECT_EQ(transmission->getGearCount(), 2) << c.node;
    }
}

TEST_F(ScriptFixture, TheControlPresetsMatchTheGearboxKinds) {
    struct Case {
        const char *node;
        bool torqueInterrupt;
        bool preselect;
        bool launchDevice;
    };

    const Case cases[] = {
        { "manual_control()", true, false, false },
        { "robotised_manual_control()", true, false, false },
        { "dual_clutch_control()", false, true, false },
        { "converter_control()", false, false, true } };

    for (const Case &c : cases) {
        delete es_script::Compiler::output()->powertrain;
        delete es_script::Compiler::output()->controlProgram;
        *es_script::Compiler::output() = es_script::Compiler::Output();

        const std::string body =
            std::string("set_powertrain(tcu: ") + c.node + ")\n";
        ASSERT_TRUE(run(body)) << c.node;

        powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
        ASSERT_NE(unit, nullptr) << c.node;

        const powertrain::TransmissionControlUnit::Parameters &params =
            unit->getTransmissionControlUnit().getParameters();

        EXPECT_EQ(params.requiresTorqueInterrupt, c.torqueInterrupt) << c.node;
        EXPECT_EQ(params.supportsPreselect, c.preselect) << c.node;
        EXPECT_EQ(params.hasLaunchDevice, c.launchDevice) << c.node;
    }
}

TEST_F(ScriptFixture, TheShiftAndLockupParametersReachTheControlUnit) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(\n"
        "        torque_cut: 0.95,\n"
        "        overlap_hold: 0.30,\n"
        "        speed_match_tolerance: 200 * units.rpm,\n"
        "        lockup_slip_target: 80 * units.rpm,\n"
        "        lockup_lock_slip: 15 * units.rpm,\n"
        "        lockup_apply_rate: 4.0,\n"
        "        lockup_controller: pid_controller(kp: 0.01, ki: 2.0, min: 0.0, max: 1.0),\n"
        "        lockup_map: map_2d()\n"
        "            .add_map_sample(x: 0.0, y: 0.0, value: 8.0)\n"
        "            .add_map_sample(x: 1.0, y: 0.0, value: 30.0)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    const powertrain::TransmissionControlUnit &tcu = unit->getTransmissionControlUnit();
    const powertrain::TransmissionControlUnit::Parameters &params = tcu.getParameters();

    EXPECT_NEAR(params.shiftTorqueCut, 0.95, 1e-12);
    EXPECT_NEAR(params.overlapHold, 0.30, 1e-12);
    EXPECT_NEAR(params.speedMatchTolerance, units::rpm(200.0), 1e-9);
    EXPECT_NEAR(params.lockupSlipTarget, units::rpm(80.0), 1e-9);
    EXPECT_NEAR(params.lockupLockSlip, units::rpm(15.0), 1e-9);
    EXPECT_NEAR(params.lockupApplyRate, 4.0, 1e-12);
    EXPECT_NEAR(params.lockupController.kp, 0.01, 1e-12);
    EXPECT_NEAR(params.lockupController.ki, 2.0, 1e-12);

    EXPECT_NEAR(
        unit->getTransmissionControlUnit().getLockupMap().sample(0.0, 0.0),
        8.0,
        1e-9);
}

TEST_F(ScriptFixture, ScriptedParameterOverridesReachTheRegistry) {
    ASSERT_TRUE(run(
        "set_powertrain(tcu: transmission_control_unit())\n"
        "set_parameter(path: \"tcu.shift.min_gear_time\", value: 0.33)\n"
        "set_map_cell(path: \"tcu.upshift_map\", x: 1, y: 2, value: 12.5)\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    const auto &overrides = es_script::Compiler::output()->parameterOverrides;
    ASSERT_EQ(overrides.size(), 2u);

    for (const auto &override : overrides) {
        EXPECT_TRUE(registry.set(override.first, override.second)) << override.first;
    }

    EXPECT_NEAR(
        unit->getTransmissionControlUnit().getParameters().minGearTime, 0.33, 1e-12);
    EXPECT_NEAR(
        unit->getTransmissionControlUnit().getUpshiftMap().getValue(1, 2), 12.5, 1e-12);
}

TEST_F(ScriptFixture, TheExportedScriptCompilesAndRestoresTheValues) {
    ASSERT_TRUE(run("set_powertrain(tcu: transmission_control_unit())\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    ASSERT_TRUE(registry.set("tcu.shift.min_gear_time", 0.42));
    ASSERT_TRUE(registry.set("tcu.upshift_map[1][2]", 33.0));

    std::ostringstream exported;
    registry.exportScript(exported, config::ParameterRegistry::ExportScope::Changed);

    const std::string body = exported.str();
    ASSERT_NE(body.find("set_parameter(\"tcu.shift.min_gear_time\""), std::string::npos);
    ASSERT_NE(body.find("set_map_cell(\"tcu.upshift_map\""), std::string::npos);

    ASSERT_TRUE(run(body)) << "the exported script does not compile";

    double minGearTime = 0.0;
    double cell = 0.0;

    for (const auto &override : es_script::Compiler::output()->parameterOverrides) {
        if (override.first == "tcu.shift.min_gear_time") minGearTime = override.second;
        if (override.first == "tcu.upshift_map[1][2]") cell = override.second;
    }

    EXPECT_NEAR(minGearTime, 0.42, 1e-6);
    EXPECT_NEAR(cell, 33.0, 1e-6);
}

TEST_F(ScriptFixture, TheExportSurvivesVeryLargeAndVerySmallValues) {
    ASSERT_TRUE(run("set_powertrain(tcu: transmission_control_unit())\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    ASSERT_TRUE(registry.set("tcu.upshift_map[0][0]", 0.000001));
    ASSERT_TRUE(registry.set("tcu.upshift_map[1][0]", 1000000.0));

    std::ostringstream exported;
    registry.exportScript(exported, config::ParameterRegistry::ExportScope::Changed);

    const std::string body = exported.str();
    EXPECT_EQ(body.find("e+"), std::string::npos)
        << "the exported script uses scientific notation, which piranha cannot parse:\n"
        << body;
    EXPECT_EQ(body.find("e-"), std::string::npos)
        << "the exported script uses scientific notation, which piranha cannot parse:\n"
        << body;

    ASSERT_TRUE(run(body)) << "the exported script does not compile:\n" << body;

    double small = -1.0;
    double large = -1.0;
    for (const auto &override : es_script::Compiler::output()->parameterOverrides) {
        if (override.first == "tcu.upshift_map[0][0]") small = override.second;
        if (override.first == "tcu.upshift_map[1][0]") large = override.second;
    }

    EXPECT_NEAR(small, 0.000001, 1e-12);
    EXPECT_NEAR(large, 1000000.0, 1e-6);
}

TEST(MrNumberTests, EveryEmittedNumberMatchesThePiranhaFloatRule) {
    for (double v : { 0.0, 1.0, -1.0, 0.5, -0.5, 0.000001, -0.000001,
                      1000000.0, 1000000000.0, 3600.0, 0.125, 1e-9 })
    {
        const std::string text = config::mrNumber(v);

        const std::size_t dot = text.find('.');
        ASSERT_NE(dot, std::string::npos) << v << " -> " << text;
        EXPECT_EQ(text.find('e'), std::string::npos) << v << " -> " << text;
        EXPECT_GT(dot, (text[0] == '-') ? 1u : 0u) << v << " -> " << text;
        EXPECT_LT(dot + 1, text.size()) << v << " -> " << text;

        for (std::size_t i = (text[0] == '-') ? 1 : 0; i < text.size(); ++i) {
            if (i == dot) continue;
            EXPECT_TRUE(text[i] >= '0' && text[i] <= '9') << v << " -> " << text;
        }
    }
}

TEST(MrNumberTests, NonFiniteValuesBecomeZeroRatherThanUnparseableText) {
    EXPECT_EQ(config::mrNumber(std::nan("")), "0.0");
    EXPECT_EQ(config::mrNumber(1.0 / 0.0), "0.0");
    EXPECT_EQ(config::mrNumber(-1.0 / 0.0), "0.0");
}

TEST_F(ScriptFixture, ADriveModeCarriesAWholeShiftMap) {
    ASSERT_TRUE(run(
        "set_powertrain(tcu: transmission_control_unit())\n"
        "add_drive_mode(drive_mode(name: \"sport\")\n"
        "    .set_map(\n"
        "        path: \"tcu.upshift_map\",\n"
        "        map: map_2d()\n"
        "            .add_map_sample(x: 0.0, y: 0.0, value: 20.0)\n"
        "            .add_map_sample(x: 1.0, y: 0.0, value: 60.0)\n"
        "            .add_map_sample(x: 0.0, y: 5.0, value: 20.0)\n"
        "            .add_map_sample(x: 1.0, y: 5.0, value: 60.0)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    config::DriveModeSet modes = es_script::Compiler::output()->driveModes;
    ASSERT_EQ(modes.getCount(), 1);
    ASSERT_EQ(modes.get(0).getMapOverrideCount(), 1);

    control::Map2d &map = unit->getTransmissionControlUnit().getUpshiftMap();
    const double before = map.sample(1.0, 0.0);

    ASSERT_TRUE(modes.select("sport", &registry));

    EXPECT_NEAR(map.sample(0.0, 0.0), 20.0, 1e-6);
    EXPECT_NEAR(map.sample(1.0, 0.0), 60.0, 1e-6)
        << "the scripted schedule did not reach the live map";
    EXPECT_NEAR(map.sample(1.0, 5.0), 60.0, 1e-6)
        << "the override did not cover every gear row";

    modes.restoreBaseline(&registry);
    EXPECT_NEAR(map.sample(1.0, 0.0), before, 1e-9)
        << "the baseline was not restored";
}

TEST_F(ScriptFixture, TwoModesGiveTheSameGearboxTwoShiftCharacters) {
    ASSERT_TRUE(run(
        "set_powertrain(tcu: transmission_control_unit(preselect: true, torque_interrupt: false))\n"
        "add_drive_mode(drive_mode(name: \"comfort\")\n"
        "    .set(\"tcu.shift.clutch_overlap_time\", 0.30)\n"
        "    .set_map(\n"
        "        path: \"tcu.overlap_shape\",\n"
        "        map: map_2d()\n"
        "            .add_map_sample(x: 0.0,  y: 0.0, value: 0.0)\n"
        "            .add_map_sample(x: 0.5,  y: 0.0, value: 0.5)\n"
        "            .add_map_sample(x: 1.0,  y: 0.0, value: 1.0)\n"
        "            .add_map_sample(x: 0.0,  y: 1.0, value: 0.0)\n"
        "            .add_map_sample(x: 0.5,  y: 1.0, value: 0.5)\n"
        "            .add_map_sample(x: 1.0,  y: 1.0, value: 1.0)))\n"
        "add_drive_mode(drive_mode(name: \"sport_plus\")\n"
        "    .set(\"tcu.shift.clutch_overlap_time\", 0.08)\n"
        "    .set_map(\n"
        "        path: \"tcu.overlap_shape\",\n"
        "        map: map_2d()\n"
        "            .add_map_sample(x: 0.0,  y: 0.0, value: 0.0)\n"
        "            .add_map_sample(x: 0.25, y: 0.0, value: 1.0)\n"
        "            .add_map_sample(x: 1.0,  y: 0.0, value: 1.0)\n"
        "            .add_map_sample(x: 0.0,  y: 1.0, value: 0.0)\n"
        "            .add_map_sample(x: 0.25, y: 1.0, value: 1.0)\n"
        "            .add_map_sample(x: 1.0,  y: 1.0, value: 1.0)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    config::DriveModeSet modes = es_script::Compiler::output()->driveModes;
    powertrain::TransmissionControlUnit &tcu = unit->getTransmissionControlUnit();

    ASSERT_TRUE(modes.select("comfort", &registry));
    const double comfortRise = tcu.getOverlapShape().sample(0.25, 0.5);
    const double comfortTime = tcu.getParameters().clutchOverlapTime;

    ASSERT_TRUE(modes.select("sport_plus", &registry));
    const double sportRise = tcu.getOverlapShape().sample(0.25, 0.5);
    const double sportTime = tcu.getParameters().clutchOverlapTime;

    EXPECT_NEAR(comfortRise, 0.25, 1e-6);
    EXPECT_NEAR(sportRise, 1.0, 1e-6)
        << "the sport shape did not reach the live map";
    EXPECT_LT(sportTime, comfortTime);

    ASSERT_TRUE(modes.select("comfort", &registry));
    EXPECT_NEAR(tcu.getOverlapShape().sample(0.25, 0.5), 0.25, 1e-6)
        << "switching back did not restore the comfort shape";
}

TEST_F(ScriptFixture, WithoutAShapeTheRampStaysLinear) {
    ASSERT_TRUE(run("set_powertrain(tcu: transmission_control_unit())\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    control::Map2d &shape =
        unit->getTransmissionControlUnit().getOverlapShape();

    for (double t = 0.0; t <= 1.0; t += 0.125) {
        EXPECT_NEAR(shape.sample(t, 0.5), t, 1e-9) << "phase " << t;
    }
}

TEST_F(ScriptFixture, TheStarterPresetBuildsAFallingTorqueCurve) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(\n"
        "        lockup_map: dc_starter_torque_map(\n"
        "            stall_torque: 180 * units.Nm,\n"
        "            free_speed: 320 * units.rpm)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    control::Map2d &map = unit->getTransmissionControlUnit().getLockupMap();

    const double warm = units::celcius(80.0);
    const double cold = units::celcius(-20.0);

    EXPECT_NEAR(map.sample(0.0, warm), units::torque(180.0, units::Nm), 1e-6);
    EXPECT_NEAR(map.sample(units::rpm(320.0), warm), 0.0, 1e-6);
    EXPECT_GT(map.sample(0.0, warm), map.sample(units::rpm(160.0), warm))
        << "the torque does not fall with speed";
    EXPECT_LT(map.sample(0.0, cold), map.sample(0.0, warm))
        << "the cold column is not weaker";
}

TEST_F(ScriptFixture, TheStarterSpeedPresetRisesWithTemperature) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(\n"
        "        lockup_map: dc_starter_speed_map(\n"
        "            cold_speed: 140 * units.rpm,\n"
        "            warm_speed: 250 * units.rpm)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    control::Map2d &map = unit->getTransmissionControlUnit().getLockupMap();

    EXPECT_NEAR(map.sample(units::celcius(80.0), 0.0), units::rpm(250.0), 1e-6);
    EXPECT_NEAR(map.sample(units::celcius(-20.0), 0.0), units::rpm(140.0), 1e-6);
}

TEST_F(ScriptFixture, TheKickdownIsScriptedAndModeDependent) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(\n"
        "        kickdown_pedal_rate: 3.0,\n"
        "        kickdown_map: map_2d()\n"
        "            .add_map_sample(x: 0.0, value: 3000 * units.rpm)\n"
        "            .add_map_sample(x: 1.0, value: 4000 * units.rpm)))\n"
        "add_drive_mode(drive_mode(name: \"comfort\")\n"
        "    .set(\"tcu.kickdown.pedal_rate\", 50.0))\n"
        "add_drive_mode(drive_mode(name: \"sport_plus\")\n"
        "    .set(\"tcu.kickdown.pedal_rate\", 2.0)\n"
        "    .set_map(path: \"tcu.kickdown_map\", map: map_2d()\n"
        "        .add_map_sample(x: 0.0, value: 6000 * units.rpm)\n"
        "        .add_map_sample(x: 1.0, value: 7000 * units.rpm)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    config::DriveModeSet modes = es_script::Compiler::output()->driveModes;
    powertrain::TransmissionControlUnit &tcu = unit->getTransmissionControlUnit();

    EXPECT_NEAR(tcu.getParameters().kickdownPedalRate, 3.0, 1e-9)
        << "the scripted pedal rate did not reach the control unit";
    EXPECT_NEAR(tcu.kickdownTarget(1.0), units::rpm(4000.0), 1e-6);

    ASSERT_TRUE(modes.select("sport_plus", &registry));
    EXPECT_NEAR(tcu.getParameters().kickdownPedalRate, 2.0, 1e-9);
    EXPECT_NEAR(tcu.kickdownTarget(1.0), units::rpm(7000.0), 1e-6)
        << "the sport schedule did not reach the live map";

    const int eager = tcu.scheduleGear(4, 1.0, 30.0, true);

    ASSERT_TRUE(modes.select("comfort", &registry));
    EXPECT_NEAR(tcu.getParameters().kickdownPedalRate, 50.0, 1e-9)
        << "comfort did not switch the stab trigger off";
    EXPECT_NEAR(tcu.kickdownTarget(1.0), units::rpm(4000.0), 1e-6)
        << "the map was not restored to the script value";

    const int calm = tcu.scheduleGear(4, 1.0, 30.0, true);

    EXPECT_LT(eager, calm) << "sport did not reach a lower gear than comfort";
}

TEST_F(ScriptFixture, ADriveModeSwitchesTheDoubleDownshiftStrategy) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: dual_clutch_control(multi_via_intermediate: false, multi_max_gears: 4))\n"
        "add_drive_mode(drive_mode(name: \"comfort\")\n"
        "    .set(\"tcu.shift.multi_via_intermediate\", 1)\n"
        "    .set_map(path: \"tcu.intermediate_bias\", map: map_2d()\n"
        "        .add_map_sample(x: 0.0, y: 0.0, value: 0.0)\n"
        "        .add_map_sample(x: 1.0, y: 0.0, value: 0.0)\n"
        "        .add_map_sample(x: 0.0, y: 9.0, value: 0.0)\n"
        "        .add_map_sample(x: 1.0, y: 9.0, value: 0.0)))\n"
        "add_drive_mode(drive_mode(name: \"sport_plus\")\n"
        "    .set(\"tcu.shift.multi_via_intermediate\", 0))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    config::DriveModeSet modes = es_script::Compiler::output()->driveModes;
    powertrain::TransmissionControlUnit &tcu = unit->getTransmissionControlUnit();

    EXPECT_FALSE(tcu.getParameters().multiShiftViaIntermediate)
        << "the script default should still interrupt";

    ASSERT_TRUE(modes.select("comfort", &registry));
    EXPECT_TRUE(tcu.getParameters().multiShiftViaIntermediate)
        << "comfort did not switch the strategy on";
    EXPECT_EQ(tcu.intermediateGear(5, 1, 0.5), 4)
        << "the scripted bias did not reach the live map";

    ASSERT_TRUE(modes.select("sport_plus", &registry));
    EXPECT_FALSE(tcu.getParameters().multiShiftViaIntermediate)
        << "sport did not switch it back off";
    EXPECT_EQ(tcu.intermediateGear(5, 1, 0.5), 2)
        << "the bias was not restored to the script value";
}

namespace {
    double mapTotal(const control::Map2d &map) {
        double total = 0.0;
        for (int i = 0; i < map.getXCount(); ++i) {
            for (int j = 0; j < map.getYCount(); ++j) total += map.getValue(i, j);
        }

        return total;
    }

    void runProgram(
        powertrain::ScriptedControlUnit *unit,
        const powertrain::PowertrainState &state,
        const powertrain::DriverInputs &inputs,
        powertrain::ActuatorCommands *commands,
        int steps = 1)
    {
        for (int i = 0; i < steps; ++i) {
            unit->update(1e-3, state, inputs, commands);
        }
    }
}

TEST_F(ScriptFixture, AScriptedProgramDrivesTheThrottleFromThePedal) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            actuator(\n"
        "                channel: \"throttle_plate\",\n"
        "                a: gain(a: signal(channel: \"accelerator\"), gain: 0.5))))\n"));

    powertrain::ScriptedControlUnit *unit =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(unit, nullptr);

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 0.8;
    runProgram(unit, state, inputs, &commands);

    EXPECT_NEAR(commands.throttlePlate, 0.4, 1e-9);
}

TEST_F(ScriptFixture, AScriptedRevLimiterCutsIgnitionAboveTheLimit) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            actuator(\n"
        "                channel: \"ignition_cut\",\n"
        "                a: greater_than(\n"
        "                    a: signal(channel: \"engine_rpm\"),\n"
        "                    b: constant(7000),\n"
        "                    band: 50)))\n"
        "        .add_output(\n"
        "            actuator(\n"
        "                channel: \"throttle_plate\",\n"
        "                a: signal(channel: \"accelerator\"))))\n"));

    powertrain::ScriptedControlUnit *unit =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(unit, nullptr);

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 1.0;

    state.engineRpm = 6000.0;
    runProgram(unit, state, inputs, &commands);
    EXPECT_NEAR(commands.ignitionCutFraction, 0.0, 1e-12);
    EXPECT_NEAR(commands.throttlePlate, 1.0, 1e-12);

    state.engineRpm = 7100.0;
    runProgram(unit, state, inputs, &commands);
    EXPECT_NEAR(commands.ignitionCutFraction, 1.0, 1e-12);

    state.engineRpm = 6980.0;
    runProgram(unit, state, inputs, &commands);
    EXPECT_NEAR(commands.ignitionCutFraction, 1.0, 1e-12);

    state.engineRpm = 6900.0;
    runProgram(unit, state, inputs, &commands);
    EXPECT_NEAR(commands.ignitionCutFraction, 0.0, 1e-12);
}

TEST_F(ScriptFixture, AScriptedIdleControllerClosesTheLoop) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            actuator(\n"
        "                channel: \"throttle_plate\",\n"
        "                a: pid(\n"
        "                    setpoint: constant(800),\n"
        "                    measurement: signal(channel: \"engine_rpm\"),\n"
        "                    controller: pid_controller(\n"
        "                        kp: 0.001, ki: 0.01, min: 0.0, max: 1.0)))))\n"));

    powertrain::ScriptedControlUnit *unit =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(unit, nullptr);

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    state.engineRpm = 600.0;
    runProgram(unit, state, inputs, &commands, 200);
    const double belowTarget = commands.throttlePlate;

    unit->reset();

    state.engineRpm = 1000.0;
    runProgram(unit, state, inputs, &commands, 200);
    const double aboveTarget = commands.throttlePlate;

    EXPECT_GT(belowTarget, aboveTarget);
    EXPECT_GT(belowTarget, 0.0);
    EXPECT_NEAR(aboveTarget, 0.0, 1e-12);
}

TEST_F(ScriptFixture, AnUndrivenActuatorKeepsItsSafeDefault) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            actuator(channel: \"throttle_plate\", a: constant(0.25))))\n"));

    powertrain::ScriptedControlUnit *unit =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(unit, nullptr);

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    state.gear = 2;
    runProgram(unit, state, inputs, &commands);

    EXPECT_NEAR(commands.throttlePlate, 0.25, 1e-12);
    EXPECT_TRUE(commands.ignitionEnabled);
    EXPECT_FALSE(commands.starterEnabled);
    EXPECT_NEAR(commands.fuelEnrichment, 1.0, 1e-12);
    EXPECT_EQ(commands.targetGear, 2);
}

TEST_F(ScriptFixture, ScriptedBlockParametersReachTheRegistry) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            actuator(\n"
        "                channel: \"throttle_plate\",\n"
        "                a: gain(\n"
        "                    name: \"pedal\",\n"
        "                    a: signal(channel: \"accelerator\"),\n"
        "                    gain: 0.5))))\n"));

    powertrain::ScriptedControlUnit *unit =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    ASSERT_TRUE(registry.contains("program.pedal.gain"));
    ASSERT_TRUE(registry.set("program.pedal.gain", 0.25));

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    inputs.accelerator = 1.0;
    runProgram(unit, state, inputs, &commands);

    EXPECT_NEAR(commands.throttlePlate, 0.25, 1e-9);
}

TEST_F(ScriptFixture, AFreelyNamedGateReachesTheTransmissionControlUnit) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(default_position: \"IDLE\")\n"
        "        .add_gate_position(\n"
        "            gate_position(name: \"REV\", engagement: \"reverse\",\n"
        "                max_entry_speed: 1.0, mode: \"reverse_thrust\"))\n"
        "        .add_gate_position(gate_position(name: \"IDLE\", engagement: \"neutral\"))\n"
        "        .add_gate_position(\n"
        "            gate_position(name: \"CLB\", engagement: \"forward\", mode: \"climb\"))\n"
        "        .add_gate_position(\n"
        "            gate_position(name: \"TOGA\", engagement: \"forward\", mode: \"toga\")))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    const powertrain::SelectorGate &gate =
        unit->getTransmissionControlUnit().getGate();

    ASSERT_EQ(gate.getCount(), 4);
    EXPECT_EQ(gate.get(0).name, "REV");
    EXPECT_EQ(gate.get(0).engagement, powertrain::GateEngagement::Reverse);
    EXPECT_NEAR(gate.get(0).maxEntrySpeed, 1.0, 1e-12);
    EXPECT_EQ(gate.get(0).mode, "reverse_thrust");

    EXPECT_EQ(gate.get(3).name, "TOGA");
    EXPECT_EQ(gate.get(3).engagement, powertrain::GateEngagement::Forward);
    EXPECT_EQ(gate.get(3).mode, "toga");

    EXPECT_EQ(unit->getPositionName(), "IDLE");
}

TEST_F(ScriptFixture, TheAutomaticGateHelperBuildsPRND) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(default_position: \"P\").automatic_gate())\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    const powertrain::SelectorGate &gate =
        unit->getTransmissionControlUnit().getGate();

    ASSERT_EQ(gate.getCount(), 4);
    EXPECT_EQ(gate.get(0).name, "P");
    EXPECT_EQ(gate.get(1).name, "R");
    EXPECT_EQ(gate.get(2).name, "N");
    EXPECT_EQ(gate.get(3).name, "D");

    EXPECT_TRUE(gate.get(0).requiresBrake);
    EXPECT_EQ(unit->getPositionName(), "P");
}

TEST_F(ScriptFixture, TheReverseRatioAndParkLockTorqueReachTheGearbox) {
    ASSERT_TRUE(run(
        "set_transmission(\n"
        "    dual_clutch_gearbox(reverse_ratio: 4.1, park_lock_torque: 9000)\n"
        "        .add_gear(3.0).add_gear(1.5))\n"));

    Transmission *transmission = es_script::Compiler::output()->transmission;
    ASSERT_NE(transmission, nullptr);

    EXPECT_NEAR(transmission->getParkLockTorque(), 9000.0, 1e-9);

    config::ParameterRegistry registry;
    transmission->registerParameters(&registry);

    double value = 0.0;
    ASSERT_TRUE(registry.get("driveline.reverse_ratio", &value));
    EXPECT_NEAR(value, 4.1, 1e-9);
}

namespace {
    powertrain::PowertrainState overlayState() {
        powertrain::PowertrainState state;
        state.coolantTemperature = units::celcius(90.0);
        state.engineRunning = true;
        state.engineSpeed = units::rpm(2500.0);
        state.engineRpm = 2500.0;
        state.gear = 1;
        state.gearCount = 6;
        state.vehicleSpeed = 20.0;

        return state;
    }
}

TEST_F(ScriptFixture, AnOverlayProgramLeavesUntouchedActuatorsAlone) {
    ASSERT_TRUE(run(
        "set_powertrain(tcu: transmission_control_unit())\n"
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            actuator(channel: \"timing_offset\", a: constant(0.25))))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    powertrain::ScriptedControlUnit *program = es_script::Compiler::output()->controlProgram;

    ASSERT_NE(unit, nullptr);
    ASSERT_NE(program, nullptr);

    program->setOverlay(true);

    powertrain::PowertrainState state = overlayState();
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.4;

    powertrain::ActuatorCommands direct;
    powertrain::ActuatorCommands overlaid;

    for (int i = 0; i < 200; ++i) {
        unit->update(1e-3, state, inputs, &direct);
    }

    unit->reset();
    program->reset();

    for (int i = 0; i < 200; ++i) {
        unit->update(1e-3, state, inputs, &overlaid);
        program->update(1e-3, state, inputs, &overlaid);
    }

    EXPECT_NEAR(overlaid.throttlePlate, direct.throttlePlate, 1e-9)
        << "the overlay changed an actuator it does not drive";
    EXPECT_NEAR(overlaid.clutchPressure[0], direct.clutchPressure[0], 1e-9);
    EXPECT_EQ(overlaid.targetGear, direct.targetGear);

    EXPECT_NEAR(overlaid.timingOffset, 0.25, 1e-9)
        << "the overlay did not take over the actuator it drives";
    EXPECT_NEAR(direct.timingOffset, 0.0, 1e-9);
}

TEST_F(ScriptFixture, AnOverlayProgramCanTakeOverAClutch) {
    ASSERT_TRUE(run(
        "set_powertrain(tcu: transmission_control_unit())\n"
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            actuator(channel: \"clutch_pressure\", a: constant(0.3))))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    powertrain::ScriptedControlUnit *program = es_script::Compiler::output()->controlProgram;

    ASSERT_NE(unit, nullptr);
    ASSERT_NE(program, nullptr);

    program->setOverlay(true);

    powertrain::PowertrainState state = overlayState();
    powertrain::DriverInputs inputs;
    inputs.accelerator = 0.4;

    powertrain::ActuatorCommands commands;
    for (int i = 0; i < 200; ++i) {
        unit->update(1e-3, state, inputs, &commands);
        program->update(1e-3, state, inputs, &commands);
    }

    EXPECT_NEAR(commands.clutchPressure[0], 0.3, 1e-9);
}

TEST_F(ScriptFixture, TheAdaptationSwitchesReachTheManager) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    adaptation: adaptation(\n"
        "        lambda_long_term_rate: 0.4,\n"
        "        throttle_learn_from_integrator: true,\n"
        "        require_unsaturated_plate: true,\n"
        "        idle_speed_margin: 450 * units.rpm))\n"));

    const adaptation::AdaptationManager::Parameters &params =
        es_script::Compiler::output()->adaptation;

    EXPECT_NEAR(params.lambdaLongTermRate, 0.4, 1e-12);
    EXPECT_TRUE(params.throttleLearnFromIntegrator);
    EXPECT_TRUE(params.conditions.requireUnsaturatedPlate);
    EXPECT_NEAR(params.idleSpeedMargin, units::rpm(450.0), 1e-9);
}

TEST_F(ScriptFixture, TheTimingMapReachesTheEngineControlUnit) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    ecu: engine_control_unit(\n"
        "        timing_map_enabled: true,\n"
        "        timing_map: map_2d()\n"
        "            .add_map_sample(x: 1000 * units.rpm, y: 0.0, value: 10 * units.deg)\n"
        "            .add_map_sample(x: 6000 * units.rpm, y: 0.0, value: 30 * units.deg)\n"
        "            .add_map_sample(x: 1000 * units.rpm, y: 200 * units.Nm, value: 6 * units.deg)\n"
        "            .add_map_sample(x: 6000 * units.rpm, y: 200 * units.Nm, value: 22 * units.deg)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    powertrain::EngineControlUnit &ecu = unit->getEngineControlUnit();

    EXPECT_TRUE(ecu.getParameters().timingMapEnabled);
    EXPECT_EQ(ecu.getTimingMap().getXCount(), 2);
    EXPECT_NEAR(
        ecu.getTimingMap().sample(units::rpm(6000.0), 0.0),
        units::angle(30.0, units::deg),
        1e-9);
}

TEST_F(ScriptFixture, TheLambdaTrimMapAndItsLoadAxisReachTheEngineControlUnit) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    ecu: engine_control_unit(\n"
        "        lambda_trim_load_manifold: true,\n"
        "        lambda_trim_map: map_2d()\n"
        "            .add_map_sample(x: 1000 * units.rpm, y: 0.0, value: 0.01)\n"
        "            .add_map_sample(x: 5000 * units.rpm, y: 0.0, value: 0.03)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    powertrain::EngineControlUnit &ecu = unit->getEngineControlUnit();

    EXPECT_TRUE(ecu.getParameters().lambdaTrimLoadIsManifold);
    EXPECT_NEAR(
        ecu.getLambdaTrimMap().sample(units::rpm(5000.0), 0.0), 0.03, 1e-12);

    powertrain::PowertrainState state;
    state.manifoldPressure = 71000.0;
    EXPECT_NEAR(ecu.lambdaTrimLoad(state), 71000.0, 1e-9);
}

TEST_F(ScriptFixture, TheKickdownMapSurvivesTheGearboxHandshakeFromAScript) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(\n"
        "        kickdown_map: map_2d()\n"
        "            .add_map_sample(x: 0.0, y: 0.0, value: 3000 * units.rpm)\n"
        "            .add_map_sample(x: 1.0, y: 0.0, value: 5200 * units.rpm)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    powertrain::TransmissionControlUnit &tcu = unit->getTransmissionControlUnit();

    double ratios[6] = { 3.6, 2.19, 1.41, 1.0, 0.83, 0.69 };
    powertrain::GearboxCapabilities caps;
    caps.gearCount = 6;
    caps.gearRatios = ratios;
    caps.finalDrive = 3.42;
    caps.tireRadius = units::distance(12.0, units::inch);
    tcu.configureGearbox(caps);

    EXPECT_NEAR(tcu.getKickdownMap().sample(1.0, 0.0), units::rpm(5200.0), 1e-6);
}

TEST_F(ScriptFixture, AShiftMapWithItsOwnPedalAxisSurvivesTheHandshake) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(\n"
        "        upshift_map: map_2d()\n"
        "            .add_map_sample(x: 0.2, y: 0, value: 9.0)\n"
        "            .add_map_sample(x: 0.9, y: 0, value: 21.0)\n"
        "            .add_map_sample(x: 0.2, y: 1, value: 14.0)\n"
        "            .add_map_sample(x: 0.9, y: 1, value: 28.0)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    powertrain::TransmissionControlUnit &tcu = unit->getTransmissionControlUnit();

    double ratios[6] = { 3.6, 2.19, 1.41, 1.0, 0.83, 0.69 };
    powertrain::GearboxCapabilities caps;
    caps.gearCount = 6;
    caps.gearRatios = ratios;
    caps.finalDrive = 3.42;
    caps.tireRadius = units::distance(12.0, units::inch);
    tcu.configureGearbox(caps);

    const control::Map2d &map = tcu.getUpshiftMap();

    EXPECT_EQ(map.getXCount(), 2);
    EXPECT_EQ(map.getYCount(), 6);
    EXPECT_NEAR(map.getXAxis(0), 0.2, 1e-12);
    EXPECT_NEAR(map.getValue(0, 0), 9.0, 1e-9);
    EXPECT_NEAR(map.getValue(1, 1), 28.0, 1e-9);
}

TEST_F(ScriptFixture, SetAdaptiveReachesTheCompilerOutput) {
    ASSERT_TRUE(run(
        "set_powertrain(tcu: transmission_control_unit())\n"
        "set_adaptive(path: \"tcu.shift.min_gear_time\", adaptive: true,\n"
        "             min: 0.1, max: 2.0)\n"));

    const auto &grants = es_script::Compiler::output()->adaptiveOverrides;
    ASSERT_EQ(grants.size(), 1u);
    EXPECT_EQ(grants[0].path, "tcu.shift.min_gear_time");
    EXPECT_TRUE(grants[0].adaptive);
    EXPECT_NEAR(grants[0].adaptMin, 0.1, 1e-12);
    EXPECT_NEAR(grants[0].adaptMax, 2.0, 1e-12);
}

TEST_F(ScriptFixture, AZoneLearnerLearnsTheCellAtTheOperatingPoint) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            zone_learner(\n"
        "                name: \"zone\",\n"
        "                target: \"tcu.lockup_map\",\n"
        "                error: constant(-1.0),\n"
        "                x: signal(channel: \"accelerator\"),\n"
        "                y: signal(channel: \"gear\"),\n"
        "                rate: 1.0)))\n"));

    powertrain::ScriptedControlUnit *program =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(program, nullptr);

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(powertrain::TransmissionControlUnit::Parameters());

    config::ParameterRegistry registry;
    tcu.registerParameters(&registry);
    program->registerParameters(&registry);

    const double before = mapTotal(tcu.getLockupMap());

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    inputs.accelerator = 1.0;
    state.gear = 2;

    powertrain::ActuatorCommands commands;
    runProgram(program, state, inputs, &commands, 500);

    EXPECT_GT(mapTotal(tcu.getLockupMap()), before);
}

TEST_F(ScriptFixture, ANonAdaptiveTargetSilentlySwallowsTheZoneLearner) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            zone_learner(\n"
        "                name: \"zone\",\n"
        "                target: \"tcu.kickdown_map\",\n"
        "                error: constant(-1.0),\n"
        "                x: signal(channel: \"accelerator\"),\n"
        "                y: signal(channel: \"gear\"),\n"
        "                rate: 1.0)))\n"));

    powertrain::ScriptedControlUnit *program =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(program, nullptr);

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(powertrain::TransmissionControlUnit::Parameters());

    config::ParameterRegistry registry;
    tcu.registerParameters(&registry);
    program->registerParameters(&registry);

    ASSERT_FALSE(registry.isAdaptive("tcu.kickdown_map"));

    const double before = mapTotal(tcu.getKickdownMap());

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    inputs.accelerator = 1.0;

    powertrain::ActuatorCommands commands;
    runProgram(program, state, inputs, &commands, 500);

    EXPECT_NEAR(mapTotal(tcu.getKickdownMap()), before, 1e-12);
}

TEST_F(ScriptFixture, TheEngageProfileIsScriptable) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(\n"
        "        engage_bins: 12,\n"
        "        engage_learning_rate: 0.15,\n"
        "        engage_smoothing: 0.4,\n"
        "        engage_limit: 0.6))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    const control::IterativeLearningControl::Parameters &params =
        unit->getTransmissionControlUnit().getEngageProfile().getParameters();

    EXPECT_EQ(params.binCount, 12);
    EXPECT_NEAR(params.learningRate, 0.15, 1e-12);
    EXPECT_NEAR(params.smoothing, 0.4, 1e-12);
    EXPECT_NEAR(params.outputMax, 0.6, 1e-12);
    EXPECT_NEAR(params.outputMin, -0.6, 1e-12);
}

TEST_F(ScriptFixture, TheTorqueModelIsScriptable) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    adaptation: adaptation(\n"
        "        torque_model_forgetting: 0.99,\n"
        "        torque_model_initial: 2.5))\n"));

    const adaptation::AdaptationManager::Parameters &params =
        es_script::Compiler::output()->adaptation;

    EXPECT_NEAR(params.torqueModel.forgettingFactor, 0.99, 1e-12);
    EXPECT_NEAR(params.torqueModel.initialEstimate, 2.5, 1e-12);
}

TEST_F(ScriptFixture, TheEnableConditionsAreScriptable) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    adaptation: adaptation(\n"
        "        require_warm: false,\n"
        "        require_steady_speed: false,\n"
        "        require_no_shift: false,\n"
        "        require_no_limiting: false,\n"
        "        minimum_speed: 900 * units.rpm))\n"));

    const adaptation::EnableConditions &c =
        es_script::Compiler::output()->adaptation.conditions;

    EXPECT_FALSE(c.requireWarm);
    EXPECT_FALSE(c.requireSteadySpeed);
    EXPECT_FALSE(c.requireNoShift);
    EXPECT_FALSE(c.requireNoLimiting);
    EXPECT_NEAR(c.minimumSpeed, units::rpm(900.0), 1e-9);
}

TEST_F(ScriptFixture, TheIdleTrimMapAndLimiterDurationAreScriptable) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    ecu: engine_control_unit(\n"
        "        limiter_duration: 0.8 * units.sec,\n"
        "        idle_trim_map: map_2d()\n"
        "            .add_map_sample(x: 250.0, y: 0.0, value: 0.05)\n"
        "            .add_map_sample(x: 380.0, y: 0.0, value: -0.02)))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    powertrain::EngineControlUnit &ecu = unit->getEngineControlUnit();

    EXPECT_NEAR(ecu.getParameters().limiterDuration, 0.8, 1e-12);
    EXPECT_EQ(ecu.getIdleTrimMap().getXCount(), 2);
    EXPECT_NEAR(ecu.getIdleTrimMap().sample(250.0, 0.0), 0.05, 1e-12);
}

TEST_F(ScriptFixture, AProgramCanCommandTheEngagement) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(actuator(channel: \"engagement\", a: constant(3.0))))\n"));

    powertrain::ScriptedControlUnit *program =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(program, nullptr);

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    ASSERT_EQ(commands.engagement, powertrain::GateEngagement::Neutral);
    runProgram(program, state, inputs, &commands);

    EXPECT_EQ(commands.engagement, powertrain::GateEngagement::Forward);
}

TEST_F(ScriptFixture, TheLearnerRateIsLiveEditable) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            learner(name: \"trim\", target: \"program.trim.rate\",\n"
        "                    error: constant(0.0), rate: 0.05)))\n"));

    powertrain::ScriptedControlUnit *program =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(program, nullptr);

    config::ParameterRegistry registry;
    program->registerParameters(&registry);

    ASSERT_TRUE(registry.contains("program.trim.rate"));
    ASSERT_TRUE(registry.contains("program.trim.threshold"));
    ASSERT_TRUE(registry.set("program.trim.rate", 0.5));

    double value = 0.0;
    ASSERT_TRUE(registry.get("program.trim.rate", &value));
    EXPECT_NEAR(value, 0.5, 1e-12);
}

TEST_F(ScriptFixture, MoreGearsThanFitAreVisiblyTruncated) {
    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(powertrain::TransmissionControlUnit::Parameters());

    std::vector<double> ratios(powertrain::MaxGears + 4, 1.0);

    powertrain::GearboxCapabilities caps;
    caps.gearCount = static_cast<int>(ratios.size());
    caps.gearRatios = ratios.data();
    caps.finalDrive = 3.42;
    caps.tireRadius = units::distance(12.0, units::inch);
    tcu.configureGearbox(caps);

    EXPECT_EQ(tcu.getParameters().gearCount, powertrain::MaxGears);
    EXPECT_EQ(tcu.getRequestedGearCount(), powertrain::MaxGears + 4);
}

TEST_F(ScriptFixture, TheEngageProfileIsAlsoLive) {
    ASSERT_TRUE(run("set_powertrain(tcu: transmission_control_unit())\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    config::ParameterRegistry registry;
    unit->registerParameters(&registry);

    ASSERT_TRUE(registry.contains("tcu.engage.learning_rate"));
    ASSERT_TRUE(registry.contains("tcu.engage.smoothing"));
    ASSERT_TRUE(registry.contains("tcu.engage.limit"));
    ASSERT_TRUE(registry.contains("tcu.kickdown.target_speed"));
    ASSERT_TRUE(registry.contains("ecu.cold_temperature"));
    ASSERT_TRUE(registry.contains("ecu.warm_temperature"));

    ASSERT_TRUE(registry.set("tcu.engage.learning_rate", 0.11));
    EXPECT_NEAR(
        unit->getTransmissionControlUnit().getEngageProfile()
            .getParameters().learningRate,
        0.11,
        1e-12);
}

TEST_F(ScriptFixture, TheRateLimitTakesALiveEditWhileItRuns) {
    ASSERT_TRUE(run(
        "set_control_program(\n"
        "    control_program()\n"
        "        .add_output(\n"
        "            actuator(\n"
        "                channel: \"throttle_plate\",\n"
        "                a: rate_limit(name: \"ramp\",\n"
        "                              a: signal(channel: \"accelerator\"),\n"
        "                              rise: 1.0, fall: 1.0))))\n"));

    powertrain::ScriptedControlUnit *program =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(program, nullptr);

    config::ParameterRegistry registry;
    program->registerParameters(&registry);

    ASSERT_TRUE(registry.contains("program.ramp.rise"));

    powertrain::PowertrainState state;
    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;

    runProgram(program, state, inputs, &commands, 10);
    ASSERT_NEAR(commands.throttlePlate, 0.0, 1e-12);

    inputs.accelerator = 1.0;
    runProgram(program, state, inputs, &commands, 100);
    const double slow = commands.throttlePlate;

    ASSERT_GT(slow, 0.0);
    ASSERT_LT(slow, 0.5);

    ASSERT_TRUE(registry.set("program.ramp.rise", 50.0));

    runProgram(program, state, inputs, &commands, 10);

    EXPECT_GT(commands.throttlePlate - slow, 0.4)
        << "the live edit to program.ramp.rise never reached the limiter";
}

TEST_F(ScriptFixture, TheLaunchSpeedAndBrakeForceReachTheSimulation) {
    ASSERT_TRUE(run(
        "set_powertrain(\n"
        "    tcu: transmission_control_unit(launch_speed: 3.0))\n"));

    powertrain::PowertrainUnit *unit = es_script::Compiler::output()->powertrain;
    ASSERT_NE(unit, nullptr);

    EXPECT_NEAR(
        unit->getTransmissionControlUnit().getParameters().launchSpeed,
        3.0,
        1e-12);
}

TEST_F(ScriptFixture, TheBrakeForceReachesTheVehicle) {
    ASSERT_TRUE(run(
        "set_vehicle(vehicle(max_brake_force: 25000))" "\n"));

    Vehicle *vehicle = es_script::Compiler::output()->vehicle;
    ASSERT_NE(vehicle, nullptr);

    EXPECT_NEAR(vehicle->getMaxBrakeForce(), 25000.0, 1e-12);
}

TEST_F(ScriptFixture, ALookupMapIsVisibleAndLearnable) {
    ASSERT_TRUE(run(
        "set_control_program(" "\n"
        "    control_program()" "\n"
        "        .add_output(" "\n"
        "            actuator(" "\n"
        "                channel: \"throttle_plate\"," "\n"
        "                a: lookup(" "\n"
        "                    name: \"shape\"," "\n"
        "                    adaptive: true, adapt_min: 0.0, adapt_max: 1.0," "\n"
        "                    x: signal(channel: \"accelerator\")," "\n"
        "                    y: constant(0.0)," "\n"
        "                    map: map_2d()" "\n"
        "                        .add_map_sample(x: 0.0, y: 0.0, value: 0.0)" "\n"
        "                        .add_map_sample(x: 1.0, y: 0.0, value: 1.0)))))" "\n"));

    powertrain::ScriptedControlUnit *program =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(program, nullptr);

    config::ParameterRegistry registry;
    program->registerParameters(&registry);

    ASSERT_TRUE(registry.contains("program.shape"))
        << "the only map a script can build is invisible";
    EXPECT_TRUE(registry.isAdaptive("program.shape"));

    control::Map2d *map = registry.findMap("program.shape");
    ASSERT_NE(map, nullptr);

    const double before = map->sample(0.0, 0.0);
    ASSERT_TRUE(registry.accumulate("program.shape", 0.0, 0.0, 0.25));

    EXPECT_GT(map->sample(0.0, 0.0), before)
        << "a zone learner could never reach a scripted lookup map";
}

TEST_F(ScriptFixture, EveryBlockKindCanCarryTheAdaptiveFlag) {
    ASSERT_TRUE(run(
        "set_control_program(" "\n"
        "    control_program()" "\n"
        "        .add_output(" "\n"
        "            actuator(" "\n"
        "                channel: \"throttle_plate\"," "\n"
        "                a: integrator(" "\n"
        "                    name: \"acc\"," "\n"
        "                    adaptive: true, adapt_min: 0.0, adapt_max: 2.0," "\n"
        "                    a: constant(0.1)))))" "\n"));

    powertrain::ScriptedControlUnit *program =
        es_script::Compiler::output()->controlProgram;
    ASSERT_NE(program, nullptr);

    config::ParameterRegistry registry;
    program->registerParameters(&registry);

    ASSERT_TRUE(registry.contains("program.acc.min"));
    EXPECT_TRUE(registry.isAdaptive("program.acc.min"))
        << "the integrator wrapper still swallows adaptive";
    EXPECT_TRUE(registry.adapt("program.acc.min", 0.1));
}

TEST_F(ScriptFixture, ACycleInTheProgramIsReported) {
    ASSERT_TRUE(run(
        "set_control_program(" "\n"
        "    control_program()" "\n"
        "        .add_output(" "\n"
        "            actuator(" "\n"
        "                channel: \"throttle_plate\"," "\n"
        "                a: delay(a: constant(1.0)))))" "\n"));

    EXPECT_TRUE(es_script::Compiler::output()->errors.empty());
}

TEST_F(ScriptFixture, AMistypedChannelIsAnErrorNotASilentZero) {
    ASSERT_TRUE(run(
        "set_control_program(" "\n"
        "    control_program()" "\n"
        "        .add_output(" "\n"
        "            actuator(" "\n"
        "                channel: \"throttle_plate\"," "\n"
        "                a: signal(channel: \"engine_sped\"))))" "\n"));

    const auto &errors = es_script::Compiler::output()->errors;
    ASSERT_FALSE(errors.empty())
        << "a typo in a channel name produced a silent constant zero";
    EXPECT_NE(errors[0].find("engine_sped"), std::string::npos) << errors[0];

    EXPECT_EQ(es_script::Compiler::output()->controlProgram, nullptr);
}

TEST_F(ScriptFixture, AMistypedActuatorIsAlsoReported) {
    ASSERT_TRUE(run(
        "set_control_program(" "\n"
        "    control_program()" "\n"
        "        .add_output(" "\n"
        "            actuator(" "\n"
        "                channel: \"throttle_plat\"," "\n"
        "                a: constant(0.5))))" "\n"));

    const auto &errors = es_script::Compiler::output()->errors;
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors[0].find("throttle_plat"), std::string::npos) << errors[0];
}

TEST_F(ScriptFixture, TheThermalModelIsScriptable) {
    ASSERT_TRUE(run(
        "set_powertrain(" "\n"
        "    thermal: thermal(" "\n"
        "        radiator: 1200.0," "\n"
        "        initial_block_temperature: (90.0 + units.K0)))" "\n"));

    const ThermalModel::Parameters &thermal =
        es_script::Compiler::output()->thermal;

    EXPECT_NEAR(thermal.radiatorConductance, 1200.0, 1e-9);
    EXPECT_NEAR(
        thermal.initialBlockTemperature, units::celcius(90.0), 1e-9)
        << "a script cannot start a warm engine";
    EXPECT_NEAR(thermal.blockThermalMass, 120000.0, 1e-9);
}

TEST_F(ScriptFixture, TheDriverModelIsScriptable) {
    ASSERT_TRUE(run(
        "set_powertrain(driver: driver(pedal_time_constant: 0.05))" "\n"));

    EXPECT_NEAR(
        es_script::Compiler::output()->driverPedalTimeConstant, 0.05, 1e-12);
    EXPECT_NEAR(
        es_script::Compiler::output()->driverClutchTimeConstant, 0.001, 1e-12);
}

namespace {
    const char *const FrictionEngineScript = R"MR(

constants constants()
impulse_response_library ir_lib()

private node wires {
    output wire1: ignition_wire();
    output wire2: ignition_wire();
}

public node friction_probe {
    alias output __out: engine;

    engine engine(
        name: "Kohler CH750",
        starter_torque: 50 * units.lb_ft,
        starter_speed: 500 * units.rpm,
        redline: 3600 * units.rpm,
        friction: friction(
            vogel_b: 948.0,
            constant_fmep: 30000.0,
            speed_factor: 4000.0,
            viscous_friction: 33.0,
            boundary_exponent: 0.25,
            heat_to_oil: 0.75)
    )

    wires wires()

    crankshaft c0(
        throw: 69 * units.mm / 2,
        flywheel_mass: 5 * units.lb,
        mass: 5 * units.lb,
        friction_torque: 10.0 * units.lb_ft,
        moment_of_inertia: 0.22986844776863666 * 0.5,
        position_x: 0.0,
        position_y: 0.0,
        tdc: constants.pi / 4
    )

    rod_journal rj0(angle: 0.0)
    c0
        .add_rod_journal(rj0)

    piston_parameters piston_params(
        mass: 400 * units.g,
        //blowby: k_28inH2O(0.1),
        compression_height: 1.0 * units.inch,
        wrist_pin_position: 0.0,
        displacement: 0.0
    )

    connecting_rod_parameters cr_params(
        mass: 300.0 * units.g,
        moment_of_inertia: 0.0015884918028487504,
        center_of_mass: 0.0,
        length: 4.0 * units.inch
    )

    cylinder_bank_parameters bank_params(
        bore: 83 * units.mm,
        deck_height: (4.0 + 1) * units.inch + 69 * units.mm / 2
    )

    intake intake(
        plenum_volume: 1.0 * units.L,
        plenum_cross_section_area: 10.0 * units.cm2,
        intake_flow_rate: k_carb(50.0),
        idle_flow_rate: k_carb(0.0),
        idle_throttle_plate_position: 0.96,
        throttle_gamma: 1.0
    )

    exhaust_system_parameters es_params(
        outlet_flow_rate: k_carb(300.0),
        primary_tube_length: 10.0 * units.inch,
        primary_flow_rate: k_carb(200.0),
        velocity_decay: 1.0,
        volume: 20.0 * units.L
    )

    exhaust_system exhaust0(
        es_params,
        audio_volume: 1.0,
        impulse_response: ir_lib.default_0
    )

    cylinder_bank b0(bank_params, angle: -45 * units.deg)
    b0
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.1)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj0,
            intake: intake,
            exhaust_system: exhaust0,
            ignition_wire: wires.wire1
        )

    cylinder_bank b1(bank_params, angle: 45.0 * units.deg)
    b1
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.1)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj0,
            intake: intake,
            exhaust_system: exhaust0,
            ignition_wire: wires.wire2
        )

    engine
        .add_cylinder_bank(b0)
        .add_cylinder_bank(b1)

    engine.add_crankshaft(c0)

    harmonic_cam_lobe lobe(
        duration_at_50_thou: 160 * units.deg,
        gamma: 1.1,
        lift: 200 * units.thou,
        steps: 100
    )

    vtwin90_camshaft_builder camshaft(
        lobe_profile: lobe,
        lobe_separation: 114 * units.deg,
        base_radius: 500 * units.thou
    )

    b0.set_cylinder_head (
        generic_small_engine_head(
            chamber_volume: 50 * units.cc,
            intake_camshaft: camshaft.intake_cam_0,
            exhaust_camshaft: camshaft.exhaust_cam_0
        )
    )
    b1.set_cylinder_head (
        generic_small_engine_head(
            chamber_volume: 50 * units.cc,
            intake_camshaft: camshaft.intake_cam_1,
            exhaust_camshaft: camshaft.exhaust_cam_1,
            flip_display: true
        )
    )

    function timing_curve(1000 * units.rpm)
    timing_curve
        .add_sample(0000 * units.rpm, 50 * units.deg)
        .add_sample(1000 * units.rpm, 50 * units.deg)
        .add_sample(2000 * units.rpm, 50 * units.deg)
        .add_sample(3000 * units.rpm, 50 * units.deg)
        .add_sample(4000 * units.rpm, 50 * units.deg)

    engine.add_ignition_module(
        vtwin90_distributor(
            wires: wires,
            timing_curve: timing_curve,
            rev_limit: 5000 * units.rpm
        ))
}

set_engine(friction_probe())
)MR";
}

TEST_F(ScriptFixture, AManoeuvreReachesTheCompilerOutput) {
    ASSERT_TRUE(run(
        "add_manoeuvre(\n"
        "    manoeuvre(name: \"wide open throttle\")\n"
        "        .at(time: 0.0, accelerator: 0.0, gate: 3)\n"
        "        .at(time: 2.0, accelerator: 1.0)\n"
        "        .at(time: 6.0, accelerator: 1.0, shift_up: true)\n"
        "        .at(time: 9.0, accelerator: 0.0, brake: 1.0))\n"));

    const auto &manoeuvres = es_script::Compiler::output()->manoeuvres;
    ASSERT_EQ(manoeuvres.size(), 1u);

    const powertrain::Manoeuvre &manoeuvre = manoeuvres[0];
    EXPECT_EQ(manoeuvre.getName(), "wide open throttle");
    EXPECT_EQ(manoeuvre.getCount(), 4);
    EXPECT_NEAR(manoeuvre.getDuration(), 9.0, 1e-9);

    EXPECT_NEAR(manoeuvre.sample(1.0).accelerator, 0.5, 1e-9);
    EXPECT_EQ(manoeuvre.sample(1.0).gatePosition, 3);
    EXPECT_TRUE(manoeuvre.sample(6.0).shiftUpRequest);
    EXPECT_FALSE(manoeuvre.sample(2.0).shiftUpRequest);
    EXPECT_NEAR(manoeuvre.sample(9.0).brake, 1.0, 1e-9);
}

TEST_F(ScriptFixture, TheSetpointsAreOrderedByTimeNotBySourceOrder) {
    ASSERT_TRUE(run(
        "add_manoeuvre(\n"
        "    manoeuvre(name: \"out of order\")\n"
        "        .at(time: 4.0, accelerator: 1.0)\n"
        "        .at(time: 0.0, accelerator: 0.0)\n"
        "        .at(time: 2.0, accelerator: 0.5))\n"));

    const auto &manoeuvres = es_script::Compiler::output()->manoeuvres;
    ASSERT_EQ(manoeuvres.size(), 1u);

    EXPECT_NEAR(manoeuvres[0].get(0).time, 0.0, 1e-9);
    EXPECT_NEAR(manoeuvres[0].get(2).time, 4.0, 1e-9);
    EXPECT_NEAR(manoeuvres[0].sample(1.0).accelerator, 0.25, 1e-9);
}

TEST_F(ScriptFixture, SeveralManoeuvresFormALibrary) {
    ASSERT_TRUE(run(
        "add_manoeuvre(manoeuvre(name: \"a\").at(time: 0.0).at(time: 1.0))\n"
        "add_manoeuvre(manoeuvre(name: \"b\").at(time: 0.0).at(time: 5.0))\n"));

    const auto &manoeuvres = es_script::Compiler::output()->manoeuvres;
    ASSERT_EQ(manoeuvres.size(), 2u);
    EXPECT_EQ(manoeuvres[0].getName(), "a");
    EXPECT_EQ(manoeuvres[1].getName(), "b");
    EXPECT_NEAR(manoeuvres[1].getDuration(), 5.0, 1e-9);
}

TEST_F(ScriptFixture, AnEmptyManoeuvreIsNotAdded) {
    ASSERT_TRUE(run("add_manoeuvre(manoeuvre(name: \"nothing\"))\n"));

    EXPECT_TRUE(es_script::Compiler::output()->manoeuvres.empty());
}

TEST_F(ScriptFixture, TooManySetpointsAreReportedAndTheManoeuvreIsRefused) {
    std::ostringstream body;
    body << "add_manoeuvre(\n    manoeuvre(name: \"far too long\")";
    for (int i = 0; i < powertrain::Manoeuvre::MaxSetpoints + 3; ++i) {
        body << "\n        .at(time: " << config::mrNumber(i * 0.01) << ")";
    }
    body << ")\n";

    ASSERT_TRUE(run(body.str()));

    const auto &errors = es_script::Compiler::output()->errors;
    ASSERT_FALSE(errors.empty()) << "the overflow must not pass silently";

    bool named = false;
    for (const std::string &error : errors) {
        if (error.find("far too long") != std::string::npos
            && error.find("dropped") != std::string::npos)
        {
            named = true;
        }
    }

    EXPECT_TRUE(named) << "the error must name the manoeuvre and say what happened";
    EXPECT_TRUE(es_script::Compiler::output()->manoeuvres.empty())
        << "a truncated manoeuvre must be refused, not silently shortened";
}

TEST_F(ScriptFixture, ARecordedManoeuvreCompilesAndReplaysTheSameShape) {
    powertrain::ManoeuvreRecorder recorder;
    recorder.setInterval(0.02);
    recorder.setTolerance(0.02);

    recorder.start(0.0);
    for (double t = 0.0; t <= 3.0; t += 0.001) {
        powertrain::DriverInputs inputs;
        inputs.clutchPedal = 0.0;
        inputs.gatePosition = 3;
        inputs.accelerator = (t < 1.0) ? t : ((t < 2.0) ? 1.0 : (3.0 - t));
        if (t >= 1.5 && t < 1.502) inputs.shiftUpRequest = true;
        recorder.update(t, inputs);
    }
    recorder.stop();

    powertrain::Manoeuvre reference;
    recorder.thin(&reference);
    ASSERT_GT(reference.getCount(), 2);

    const std::string script = recorder.toScript("recorded");
    ASSERT_TRUE(run(script)) << "the recorded manoeuvre does not compile:\n" << script;

    const auto &manoeuvres = es_script::Compiler::output()->manoeuvres;
    ASSERT_EQ(manoeuvres.size(), 1u);
    EXPECT_EQ(manoeuvres[0].getName(), "recorded");
    EXPECT_EQ(manoeuvres[0].getCount(), reference.getCount());

    for (double t = 0.0; t <= 3.0; t += 0.01) {
        EXPECT_NEAR(
            manoeuvres[0].sample(t).accelerator,
            reference.sample(t).accelerator,
            1e-6) << t;
    }

    bool sawShift = false;
    for (int i = 0; i < manoeuvres[0].getCount(); ++i) {
        if (manoeuvres[0].get(i).shiftUp) sawShift = true;
    }

    EXPECT_TRUE(sawShift) << "the shift request must survive the round trip";
}

TEST_F(ScriptFixture, TheFrictionNodeReachesTheEngine) {
    ASSERT_TRUE(run(FrictionEngineScript));

    Engine *engine = es_script::Compiler::output()->engine;
    ASSERT_NE(engine, nullptr);

    const EngineFriction::Parameters &params =
        engine->getFrictionModel().getParameters();

    EXPECT_NEAR(params.vogelB, 948.0, 1e-9);
    EXPECT_NEAR(params.constantFmep, 30000.0, 1e-9);
    EXPECT_NEAR(params.speedFactor, 4000.0, 1e-9);
    EXPECT_NEAR(params.frictionHeatToOil, 0.75, 1e-9);
}

TEST_F(ScriptFixture, TheCylinderFrictionReachesEveryChamber) {
    ASSERT_TRUE(run(FrictionEngineScript));

    Engine *engine = es_script::Compiler::output()->engine;
    ASSERT_NE(engine, nullptr);
    ASSERT_GT(engine->getCylinderCount(), 0);

    EXPECT_NEAR(engine->getCylinderFriction().viscousFrictionCoefficient, 33.0, 1e-9);
    EXPECT_NEAR(engine->getCylinderFriction().boundaryExponent, 0.25, 1e-9);

    for (int i = 0; i < engine->getCylinderCount(); ++i) {
        EXPECT_NEAR(
            engine->getChamber(i)->getFrictionModel().viscousFrictionCoefficient,
            33.0,
            1e-9) << i;
    }
}

TEST_F(ScriptFixture, TheScriptedViscosityChangesTheChamberFriction) {
    ASSERT_TRUE(run(FrictionEngineScript));

    Engine *engine = es_script::Compiler::output()->engine;
    ASSERT_NE(engine, nullptr);

    engine->getThermalModel().setOilTemperature(units::celcius(100.0));
    engine->updateFriction(1e-3);
    const double warm = engine->getViscosityRatio();

    engine->getThermalModel().setOilTemperature(units::celcius(20.0));
    engine->updateFriction(1e-3);
    const double cold = engine->getViscosityRatio();

    EXPECT_NEAR(warm, 1.0, 1e-6);
    EXPECT_GT(cold, 10.0);
    EXPECT_NEAR(engine->getChamber(0)->frictionForce(5.0, 0.0), 33.0 * cold * 5.0, 1e-6);
}

namespace {
    const char *const PlainEngineScript = R"MR(

constants constants()
impulse_response_library ir_lib()

private node wires {
    output wire1: ignition_wire();
    output wire2: ignition_wire();
}

public node plain_probe {
    alias output __out: engine;

    engine engine(
        name: "Kohler CH750",
        starter_torque: 50 * units.lb_ft,
        starter_speed: 500 * units.rpm,
        redline: 3600 * units.rpm
    )

    wires wires()

    crankshaft c0(
        throw: 69 * units.mm / 2,
        flywheel_mass: 5 * units.lb,
        mass: 5 * units.lb,
        friction_torque: 10.0 * units.lb_ft,
        moment_of_inertia: 0.22986844776863666 * 0.5,
        position_x: 0.0,
        position_y: 0.0,
        tdc: constants.pi / 4
    )

    rod_journal rj0(angle: 0.0)
    c0
        .add_rod_journal(rj0)

    piston_parameters piston_params(
        mass: 400 * units.g,
        //blowby: k_28inH2O(0.1),
        compression_height: 1.0 * units.inch,
        wrist_pin_position: 0.0,
        displacement: 0.0
    )

    connecting_rod_parameters cr_params(
        mass: 300.0 * units.g,
        moment_of_inertia: 0.0015884918028487504,
        center_of_mass: 0.0,
        length: 4.0 * units.inch
    )

    cylinder_bank_parameters bank_params(
        bore: 83 * units.mm,
        deck_height: (4.0 + 1) * units.inch + 69 * units.mm / 2
    )

    intake intake(
        plenum_volume: 1.0 * units.L,
        plenum_cross_section_area: 10.0 * units.cm2,
        intake_flow_rate: k_carb(50.0),
        idle_flow_rate: k_carb(0.0),
        idle_throttle_plate_position: 0.96,
        throttle_gamma: 1.0
    )

    exhaust_system_parameters es_params(
        outlet_flow_rate: k_carb(300.0),
        primary_tube_length: 10.0 * units.inch,
        primary_flow_rate: k_carb(200.0),
        velocity_decay: 1.0,
        volume: 20.0 * units.L
    )

    exhaust_system exhaust0(
        es_params,
        audio_volume: 1.0,
        impulse_response: ir_lib.default_0
    )

    cylinder_bank b0(bank_params, angle: -45 * units.deg)
    b0
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.1)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj0,
            intake: intake,
            exhaust_system: exhaust0,
            ignition_wire: wires.wire1
        )

    cylinder_bank b1(bank_params, angle: 45.0 * units.deg)
    b1
        .add_cylinder(
            piston: piston(piston_params, blowby: k_28inH2O(0.1)),
            connecting_rod: connecting_rod(cr_params),
            rod_journal: rj0,
            intake: intake,
            exhaust_system: exhaust0,
            ignition_wire: wires.wire2
        )

    engine
        .add_cylinder_bank(b0)
        .add_cylinder_bank(b1)

    engine.add_crankshaft(c0)

    harmonic_cam_lobe lobe(
        duration_at_50_thou: 160 * units.deg,
        gamma: 1.1,
        lift: 200 * units.thou,
        steps: 100
    )

    vtwin90_camshaft_builder camshaft(
        lobe_profile: lobe,
        lobe_separation: 114 * units.deg,
        base_radius: 500 * units.thou
    )

    b0.set_cylinder_head (
        generic_small_engine_head(
            chamber_volume: 50 * units.cc,
            intake_camshaft: camshaft.intake_cam_0,
            exhaust_camshaft: camshaft.exhaust_cam_0
        )
    )
    b1.set_cylinder_head (
        generic_small_engine_head(
            chamber_volume: 50 * units.cc,
            intake_camshaft: camshaft.intake_cam_1,
            exhaust_camshaft: camshaft.exhaust_cam_1,
            flip_display: true
        )
    )

    function timing_curve(1000 * units.rpm)
    timing_curve
        .add_sample(0000 * units.rpm, 50 * units.deg)
        .add_sample(1000 * units.rpm, 50 * units.deg)
        .add_sample(2000 * units.rpm, 50 * units.deg)
        .add_sample(3000 * units.rpm, 50 * units.deg)
        .add_sample(4000 * units.rpm, 50 * units.deg)

    engine.add_ignition_module(
        vtwin90_distributor(
            wires: wires,
            timing_curve: timing_curve,
            rev_limit: 5000 * units.rpm
        ))
}

set_engine(plain_probe())
)MR";

    class ConstraintRig : public PistonEngineSimulator {
        public:
            void prepare(Engine *target) {
                m_engine = target;
                m_crankshaftFrictionConstraints =
                    new atg_scs::RotationFrictionConstraint[target->getCrankshaftCount()];
            }

            void release() {
                delete[] m_crankshaftFrictionConstraints;
                m_crankshaftFrictionConstraints = nullptr;
                m_engine = nullptr;
            }

            using PistonEngineSimulator::updateFrictionConstraints;

            double maxTorque(int i) const {
                return m_crankshaftFrictionConstraints[i].m_maxTorque;
            }

            double minTorque(int i) const {
                return m_crankshaftFrictionConstraints[i].m_minTorque;
            }

            virtual void writeToSynthesizer() override { /* void */ }
    };
}

TEST_F(ScriptFixture, TheCrankFrictionTorqueReachesTheConstraint) {
    ASSERT_TRUE(run(FrictionEngineScript));

    Engine *engine = es_script::Compiler::output()->engine;
    ASSERT_NE(engine, nullptr);
    ASSERT_GT(engine->getCrankshaftCount(), 0);

    ConstraintRig rig;
    rig.prepare(engine);

    engine->getThermalModel().setOilTemperature(units::celcius(100.0));
    engine->getCrankshaft(0)->m_body.v_theta = units::rpm(3000.0);
    engine->updateFriction(1e-3);
    rig.updateFrictionConstraints();

    const double scripted = engine->getCrankshaft(0)->getFrictionTorque();
    const double chenFlynn = engine->getCrankFrictionTorque();

    EXPECT_GT(chenFlynn, 0.0);
    EXPECT_NEAR(rig.maxTorque(0), scripted + chenFlynn, 1e-9);
    EXPECT_NEAR(rig.minTorque(0), -(scripted + chenFlynn), 1e-9);

    rig.release();
}

TEST_F(ScriptFixture, TheConstraintRisesWithEngineSpeed) {
    ASSERT_TRUE(run(FrictionEngineScript));

    Engine *engine = es_script::Compiler::output()->engine;
    ASSERT_NE(engine, nullptr);

    ConstraintRig rig;
    rig.prepare(engine);
    engine->getThermalModel().setOilTemperature(units::celcius(100.0));

    engine->getCrankshaft(0)->m_body.v_theta = units::rpm(1000.0);
    engine->updateFriction(1e-3);
    rig.updateFrictionConstraints();
    const double slow = rig.maxTorque(0);

    engine->getCrankshaft(0)->m_body.v_theta = units::rpm(6000.0);
    engine->updateFriction(1e-3);
    rig.updateFrictionConstraints();
    const double fast = rig.maxTorque(0);

    EXPECT_GT(fast, slow);

    rig.release();
}

TEST_F(ScriptFixture, WithoutAFrictionNodeTheConstraintKeepsTheScriptedTorqueExactly) {
    ASSERT_TRUE(run(PlainEngineScript));

    Engine *engine = es_script::Compiler::output()->engine;
    ASSERT_NE(engine, nullptr);

    ConstraintRig rig;
    rig.prepare(engine);

    for (double rpm : { 0.0, 800.0, 3000.0, 6000.0 }) {
        for (double oil : { -20.0, 20.0, 90.0, 150.0 }) {
            engine->getThermalModel().setOilTemperature(units::celcius(oil));
            engine->getCrankshaft(0)->m_body.v_theta = units::rpm(rpm);
            engine->updateFriction(1e-3);
            rig.updateFrictionConstraints();

            EXPECT_EQ(engine->getCrankFrictionTorque(), 0.0) << rpm << " " << oil;
            EXPECT_EQ(engine->getViscosityRatio(), 1.0) << rpm << " " << oil;
            EXPECT_EQ(
                rig.maxTorque(0),
                engine->getCrankshaft(0)->getFrictionTorque()) << rpm << " " << oil;
        }
    }

    rig.release();
}

TEST_F(ScriptFixture, WithoutAFrictionNodeTheCylinderFrictionKeepsItsOldValues) {
    ASSERT_TRUE(run(PlainEngineScript));

    Engine *engine = es_script::Compiler::output()->engine;
    ASSERT_NE(engine, nullptr);

    const CombustionChamber::FrictionModelParams &params = engine->getCylinderFriction();

    EXPECT_EQ(params.frictionCoeff, 0.06);
    EXPECT_EQ(params.breakawayFriction, units::force(50.0, units::N));
    EXPECT_EQ(params.breakawayFrictionVelocity, 0.1);
    EXPECT_EQ(params.viscousFrictionCoefficient, 20.0);
    EXPECT_EQ(params.boundaryExponent, 0.0);

    engine->getThermalModel().setOilTemperature(units::celcius(20.0));
    engine->updateFriction(1e-3);

    const double reference =
        engine->getChamber(0)->frictionForce(5.0, 400.0);

    engine->getThermalModel().setOilTemperature(units::celcius(140.0));
    engine->updateFriction(1e-3);

    EXPECT_EQ(engine->getChamber(0)->frictionForce(5.0, 400.0), reference);
}

TEST_F(ScriptFixture, TheCrankFrictionWorkWarmsTheOil) {
    ASSERT_TRUE(run(FrictionEngineScript));

    Engine *engine = es_script::Compiler::output()->engine;
    ASSERT_NE(engine, nullptr);

    ThermalModel::Parameters thermal;
    thermal.blockThermalMass = 1000.0;
    thermal.oilThermalMass = 500.0;
    thermal.blockToOilConductance = 0.0;
    thermal.radiatorConductance = 0.0;
    thermal.oilToAmbientConductance = 0.0;
    thermal.oilCoolerConductance = 0.0;
    thermal.speedCoolingCoefficient = 0.0;
    thermal.combustionHeatFraction = 0.0;
    thermal.initialBlockTemperature = units::celcius(100.0);
    thermal.initialOilTemperature = units::celcius(100.0);
    engine->getThermalModel().initialize(thermal);

    engine->getCrankshaft(0)->m_body.v_theta = units::rpm(6000.0);
    engine->updateFriction(0.01);

    const double oilBefore = engine->getOilTemperature();
    const double blockBefore = engine->getCoolantTemperature();

    engine->updateThermal(0.01, 0.0);

    EXPECT_GT(engine->getFrictionPower(), 0.0);
    EXPECT_GT(engine->getOilTemperature(), oilBefore);
    EXPECT_GT(engine->getCoolantTemperature(), blockBefore);
}

TEST_F(ScriptFixture, AllFrictionHeatCanBeSentToTheOil) {
    ASSERT_TRUE(run(FrictionEngineScript));

    Engine *engine = es_script::Compiler::output()->engine;
    ASSERT_NE(engine, nullptr);
    engine->getFrictionModel().getParameters().frictionHeatToOil = 1.0;

    ThermalModel::Parameters thermal;
    thermal.blockThermalMass = 1000.0;
    thermal.oilThermalMass = 500.0;
    thermal.blockToOilConductance = 0.0;
    thermal.radiatorConductance = 0.0;
    thermal.oilToAmbientConductance = 0.0;
    thermal.oilCoolerConductance = 0.0;
    thermal.speedCoolingCoefficient = 0.0;
    thermal.combustionHeatFraction = 0.0;
    thermal.initialBlockTemperature = units::celcius(100.0);
    thermal.initialOilTemperature = units::celcius(100.0);
    engine->getThermalModel().initialize(thermal);

    engine->getCrankshaft(0)->m_body.v_theta = units::rpm(6000.0);
    engine->updateFriction(0.01);

    const double blockBefore = engine->getCoolantTemperature();
    engine->updateThermal(0.01, 0.0);

    EXPECT_GT(engine->getOilTemperature(), units::celcius(100.0));
    EXPECT_EQ(engine->getCoolantTemperature(), blockBefore);
}
