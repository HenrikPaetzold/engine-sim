#include <gtest/gtest.h>

#include "../scripting/include/compiler.h"

#include "../include/powertrain/scripted_control_unit.h"
#include "../include/transmission.h"
#include "../include/config/parameter_registry.h"
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
