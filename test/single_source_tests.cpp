#include <gtest/gtest.h>

#include "../include/config/parameter_registry.h"
#include "../include/config/drive_mode.h"
#include "../include/powertrain/engine_control_unit.h"
#include "../include/powertrain/transmission_control_unit.h"
#include "../include/powertrain/powertrain_unit.h"
#include "../include/adaptation/adaptation_manager.h"
#include "../include/ignition_module.h"
#include "../include/thermal_model.h"
#include "../include/vehicle.h"
#include "../include/units.h"

#include <sstream>
#include <string>
#include <vector>

namespace {
    powertrain::PowertrainState runningState() {
        powertrain::PowertrainState state;
        state.coolantTemperature = units::celcius(90.0);
        state.engineSpeed = units::rpm(3000.0);
        state.engineRunning = true;
        state.gear = 2;

        return state;
    }
}

TEST(SingleRevLimitTests, EcuCommandsTheHardLimiter) {
    powertrain::EngineControlUnit::Parameters params;
    params.revLimit = units::rpm(7000.0);
    params.hardLimitOffset = units::rpm(150.0);

    powertrain::EngineControlUnit ecu;
    ecu.initialize(params);

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainState state = runningState();

    ecu.update(1e-3, state, inputs, &commands);

    EXPECT_NEAR(commands.revLimit, units::rpm(7150.0), 1e-6);
    EXPECT_GT(commands.limiterDuration, 0.0);
}

TEST(SingleRevLimitTests, ChangingTheParameterMovesTheCommandedLimit) {
    config::ParameterRegistry registry;

    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());
    ecu.registerParameters(&registry, "");

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainState state = runningState();

    ecu.update(1e-3, state, inputs, &commands);
    const double before = commands.revLimit;

    ASSERT_TRUE(registry.set("ecu.limiter.rev_limit", units::rpm(9000.0)));
    ecu.update(1e-3, state, inputs, &commands);

    EXPECT_GT(commands.revLimit, before);
    EXPECT_NEAR(
        commands.revLimit,
        units::rpm(9000.0) + ecu.getParameters().hardLimitOffset,
        1e-6);
}

TEST(SingleRevLimitTests, ADriveModeMovesTheHardLimiterToo) {
    config::ParameterRegistry registry;

    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());
    ecu.registerParameters(&registry, "");

    config::DriveMode track("track");
    track.set("ecu.limiter.rev_limit", units::rpm(8500.0));

    config::DriveModeSet modes;
    modes.add(track);
    ASSERT_TRUE(modes.select("track", &registry));

    powertrain::DriverInputs inputs;
    powertrain::ActuatorCommands commands;
    powertrain::PowertrainState state = runningState();

    ecu.update(1e-3, state, inputs, &commands);

    EXPECT_NEAR(
        commands.revLimit,
        units::rpm(8500.0) + ecu.getParameters().hardLimitOffset,
        1e-6);
}

TEST(SingleRevLimitTests, TheIgnitionModuleAcceptsTheCommandedLimit) {
    IgnitionModule ignition;
    ignition.setRevLimit(units::rpm(6000.0));
    ASSERT_NEAR(ignition.getRevLimit(), units::rpm(6000.0), 1e-9);

    ignition.setRevLimit(units::rpm(9150.0));
    EXPECT_NEAR(ignition.getRevLimit(), units::rpm(9150.0), 1e-9);
}

namespace {
    std::vector<std::string> registeredPaths(const config::ParameterRegistry &registry) {
        std::vector<std::string> paths;
        for (int i = 0; i < registry.getCount(); ++i) {
            paths.push_back(registry.getDescriptor(i).path);
        }

        return paths;
    }

    bool has(const config::ParameterRegistry &registry, const char *path) {
        return registry.contains(path);
    }
}

TEST(RegistryCoverageTests, EveryVehicleParameterIsReachable) {
    Vehicle::Parameters params;
    params.mass = 1400.0;
    params.dragCoefficient = 0.30;
    params.crossSectionArea = 2.2;
    params.diffRatio = 3.9;
    params.tireRadius = 0.31;
    params.rollingResistance = 250.0;

    Vehicle vehicle;
    vehicle.initialize(params);

    config::ParameterRegistry registry;
    vehicle.registerParameters(&registry, "");

    for (const char *path : {
        "vehicle.mass",
        "vehicle.drag_coefficient",
        "vehicle.cross_section_area",
        "vehicle.diff_ratio",
        "vehicle.tire_radius",
        "vehicle.rolling_resistance",
        "vehicle.road_grade",
        "vehicle.max_brake_force" })
    {
        EXPECT_TRUE(has(registry, path)) << path;
    }

    EXPECT_EQ(registry.getCount(), 8);
}

TEST(RegistryCoverageTests, EveryThermalParameterIsReachable) {
    ThermalModel model;
    model.initialize(ThermalModel::Parameters());

    config::ParameterRegistry registry;
    model.registerParameters(&registry, "");

    for (const char *path : {
        "thermal.block_mass",
        "thermal.oil_mass",
        "thermal.block_to_oil",
        "thermal.radiator",
        "thermal.oil_to_ambient",
        "thermal.speed_cooling",
        "thermal.thermostat_open",
        "thermal.thermostat_full",
        "thermal.combustion_heat_fraction",
        "thermal.ambient_temperature" })
    {
        EXPECT_TRUE(has(registry, path)) << path;
    }

    EXPECT_EQ(registry.getCount(), 10);
}

TEST(RegistryCoverageTests, VehicleParametersWriteThrough) {
    Vehicle::Parameters params;
    params.mass = 1400.0;
    params.dragCoefficient = 0.30;
    params.crossSectionArea = 2.2;
    params.diffRatio = 3.9;
    params.tireRadius = 0.31;
    params.rollingResistance = 250.0;

    Vehicle vehicle;
    vehicle.initialize(params);

    config::ParameterRegistry registry;
    vehicle.registerParameters(&registry, "");

    ASSERT_TRUE(registry.set("vehicle.mass", 1850.0));
    EXPECT_NEAR(vehicle.getMass(), 1850.0, 1e-9);

    ASSERT_TRUE(registry.set("vehicle.road_grade", 0.07));
    EXPECT_NEAR(vehicle.getRoadGrade(), 0.07, 1e-9);
}

TEST(RegistryCoverageTests, ThermalParametersWriteThrough) {
    ThermalModel model;
    model.initialize(ThermalModel::Parameters());

    config::ParameterRegistry registry;
    model.registerParameters(&registry, "");

    ASSERT_TRUE(registry.set("thermal.ambient_temperature", units::celcius(-15.0)));
    EXPECT_NEAR(
        model.getParameters().ambientTemperature,
        units::celcius(-15.0),
        1e-9);
}

TEST(RegistryCoverageTests, TheLimiterAndCrankingFieldsAreReachable) {
    config::ParameterRegistry registry;

    powertrain::EngineControlUnit ecu;
    ecu.initialize(powertrain::EngineControlUnit::Parameters());
    ecu.registerParameters(&registry, "");

    for (const char *path : {
        "ecu.limiter.hard_offset",
        "ecu.limiter.duration",
        "ecu.cranking_speed",
        "ecu.torque.pid.kd" })
    {
        EXPECT_TRUE(has(registry, path)) << path;
    }
}

TEST(RegistryCoverageTests, TheGearboxFieldsAreReachable) {
    config::ParameterRegistry registry;

    powertrain::TransmissionControlUnit tcu;
    tcu.initialize(powertrain::TransmissionControlUnit::Parameters());
    tcu.registerParameters(&registry, "");

    for (const char *path : {
        "tcu.gearbox.final_drive",
        "tcu.gearbox.tire_radius",
        "tcu.launch.pid.kd",
        "tcu.launch.stall_protect_speed" })
    {
        EXPECT_TRUE(has(registry, path)) << path;
    }
}

namespace {
    config::ParameterRegistry buildRegistry(double *a, double *b, double *learned) {
        config::ParameterRegistry registry;

        registry.registerScalar(
            config::describeScalar("ecu.a", 0.0, 100.0, 10.0), a);
        registry.registerScalar(
            config::describeScalar("ecu.b", 0.0, 100.0, 20.0), b);

        config::ParameterDescriptor adaptive =
            config::describeScalar("ecu.learned", -1.0, 1.0, 0.0);
        adaptive.adaptive = true;
        adaptive.adaptMin = -1.0;
        adaptive.adaptMax = 1.0;
        registry.registerScalar(adaptive, learned);

        return registry;
    }
}

TEST(ExportScopeTests, LearnedScopeOnlyCoversAdaptiveValues) {
    double a = 0.0, b = 0.0, learned = 0.0;
    config::ParameterRegistry registry = buildRegistry(&a, &b, &learned);

    registry.set("ecu.a", 55.0);
    registry.adapt("ecu.learned", 0.25);

    std::ostringstream out;
    registry.exportScript(out, config::ParameterRegistry::ExportScope::Learned);

    EXPECT_NE(out.str().find("ecu.learned"), std::string::npos);
    EXPECT_EQ(out.str().find("ecu.a"), std::string::npos);
}

TEST(ExportScopeTests, ChangedScopeCoversEveryDeviation) {
    double a = 0.0, b = 0.0, learned = 0.0;
    config::ParameterRegistry registry = buildRegistry(&a, &b, &learned);

    registry.set("ecu.a", 55.0);

    std::ostringstream out;
    registry.exportScript(out, config::ParameterRegistry::ExportScope::Changed);

    EXPECT_NE(out.str().find("set_parameter(\"ecu.a\", 55)"), std::string::npos);
    EXPECT_EQ(out.str().find("ecu.b"), std::string::npos);
}

TEST(ExportScopeTests, UntouchedRegistryExportsNothing) {
    double a = 0.0, b = 0.0, learned = 0.0;
    config::ParameterRegistry registry = buildRegistry(&a, &b, &learned);

    std::ostringstream out;
    registry.exportScript(out, config::ParameterRegistry::ExportScope::Changed);

    EXPECT_TRUE(out.str().empty());
}

TEST(ExportScopeTests, OverridesRoundTripThroughTheRegistry) {
    double a = 0.0, b = 0.0, learned = 0.0;
    config::ParameterRegistry registry = buildRegistry(&a, &b, &learned);

    registry.set("ecu.a", 55.0);
    registry.set("ecu.b", 77.5);

    std::ostringstream out;
    registry.exportScript(out, config::ParameterRegistry::ExportScope::Changed);
    const std::string exported = out.str();

    registry.resetToDefaults();
    ASSERT_NEAR(a, 10.0, 1e-12);
    ASSERT_NEAR(b, 20.0, 1e-12);

    std::istringstream lines(exported);
    std::string line;
    int applied = 0;

    while (std::getline(lines, line)) {
        const size_t openQuote = line.find('"');
        const size_t closeQuote = line.find('"', openQuote + 1);
        const size_t comma = line.find(", ", closeQuote);
        const size_t close = line.rfind(')');
        if (openQuote == std::string::npos || comma == std::string::npos) continue;

        const std::string path = line.substr(openQuote + 1, closeQuote - openQuote - 1);
        const double value = std::stod(line.substr(comma + 2, close - comma - 2));

        if (registry.set(path, value)) ++applied;
    }

    EXPECT_EQ(applied, 2);
    EXPECT_NEAR(a, 55.0, 1e-9);
    EXPECT_NEAR(b, 77.5, 1e-9);
}
