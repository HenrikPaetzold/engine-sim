#include <gtest/gtest.h>

#include "../scripting/include/compiler.h"

#include "../include/powertrain/scripted_control_unit.h"
#include "../include/transmission.h"
#include "../include/engine.h"
#include "../include/config/parameter_registry.h"
#include "../include/units.h"

#include <fstream>
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
    unit->registerParameters(&registry, "");

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
    unit->registerParameters(&registry, "");

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
    transmission->registerParameters(&registry, "");

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
    unit->registerParameters(&registry, "");

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
    unit->registerParameters(&registry, "");

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
    unit->registerParameters(&registry, "");

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
    unit->registerParameters(&registry, "");

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
    unit->registerParameters(&registry, "");

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
    unit->registerParameters(&registry, "");

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
    unit->registerParameters(&registry, "");

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
    transmission->registerParameters(&registry, "");

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
