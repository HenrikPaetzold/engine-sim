#include <gtest/gtest.h>

#include "../include/powertrain_bootstrap.h"
#include "../include/powertrain_system.h"
#include "../include/control/control_program.h"
#include "../include/units.h"

#include <string>
#include <vector>

namespace {
    powertrain::PowertrainUnit *makeUnit() {
        powertrain::EngineControlUnit::Parameters engineParams;
        powertrain::TransmissionControlUnit::Parameters transmissionParams;

        powertrain::PowertrainUnit *unit = new powertrain::PowertrainUnit;
        unit->initialize(engineParams, transmissionParams);

        return unit;
    }

    powertrain::ScriptedControlUnit *makeProgram() {
        powertrain::ScriptedControlUnit *program = new powertrain::ScriptedControlUnit;
        program->initialize();

        control::ConstantBlock *constant = new control::ConstantBlock;
        constant->m_value = 0.5;
        constant->m_name = "half";
        program->getProgram().addBlock(constant);

        return program;
    }

    double value(const config::ParameterRegistry &registry, const std::string &path) {
        double value = 0.0;
        registry.get(path, &value);

        return value;
    }

    struct Fixture {
        PowertrainSystem system;
        config::ParameterRegistry registry;
        config::DriveModeSet modes;
        adaptation::AdaptationManager adaptation;

        powertrain::BootstrapContext context() {
            powertrain::BootstrapContext ctx;
            ctx.system = &system;
            ctx.registry = &registry;
            ctx.modes = &modes;
            ctx.adaptation = &adaptation;

            return ctx;
        }
    };
}

TEST(PowertrainBootstrapTests, OnlyTheUnitLeavesNoOverlayAndKeepsAdaptation) {
    Fixture fixture;

    powertrain::BootstrapInputs inputs;
    inputs.unit = makeUnit();

    const powertrain::BootstrapResult result =
        powertrain::installPowertrain(inputs, fixture.context());

    EXPECT_EQ(result.controller, static_cast<powertrain::PowertrainController *>(inputs.unit));
    EXPECT_EQ(result.overlay, nullptr);
    EXPECT_TRUE(result.adaptationAttached);
    EXPECT_EQ(fixture.system.getController(), result.controller);

    delete inputs.unit;
}

TEST(PowertrainBootstrapTests, OnlyTheProgramBecomesTheControllerWithoutAdaptation) {
    Fixture fixture;

    powertrain::BootstrapInputs inputs;
    inputs.program = makeProgram();

    const powertrain::BootstrapResult result =
        powertrain::installPowertrain(inputs, fixture.context());

    EXPECT_EQ(result.controller, static_cast<powertrain::PowertrainController *>(inputs.program));
    EXPECT_EQ(result.overlay, nullptr);
    EXPECT_FALSE(result.adaptationAttached);
    EXPECT_FALSE(inputs.program->isOverlay());

    delete inputs.program;
}

TEST(PowertrainBootstrapTests, BothTogetherMakeTheProgramAnOverlay) {
    Fixture fixture;

    powertrain::BootstrapInputs inputs;
    inputs.unit = makeUnit();
    inputs.program = makeProgram();

    const powertrain::BootstrapResult result =
        powertrain::installPowertrain(inputs, fixture.context());

    EXPECT_EQ(result.controller, static_cast<powertrain::PowertrainController *>(inputs.unit));
    EXPECT_EQ(result.overlay, static_cast<powertrain::PowertrainController *>(inputs.program));
    EXPECT_TRUE(result.adaptationAttached);
    EXPECT_TRUE(inputs.program->isOverlay());

    delete inputs.unit;
    delete inputs.program;
}

TEST(PowertrainBootstrapTests, BothControllersRegisterTheirParameters) {
    Fixture fixture;

    powertrain::BootstrapInputs inputs;
    inputs.unit = makeUnit();
    inputs.program = makeProgram();

    powertrain::installPowertrain(inputs, fixture.context());

    EXPECT_TRUE(fixture.registry.contains("ecu.limiter.rev_limit"));
    EXPECT_TRUE(fixture.registry.contains("program.half"));

    delete inputs.unit;
    delete inputs.program;
}

TEST(PowertrainBootstrapTests, ScriptOverridesLandAfterTheRegistryIsBuilt) {
    Fixture fixture;

    powertrain::BootstrapInputs inputs;
    inputs.unit = makeUnit();
    inputs.parameterOverrides.push_back({ "ecu.limiter.rev_limit", units::rpm(7600.0) });

    powertrain::installPowertrain(inputs, fixture.context());

    EXPECT_NEAR(value(fixture.registry, "ecu.limiter.rev_limit"), units::rpm(7600.0), 1e-6);

    delete inputs.unit;
}

TEST(PowertrainBootstrapTests, TheDefaultModeWinsOverAScriptOverride) {
    Fixture fixture;

    config::DriveMode sport("sport");
    sport.set("ecu.limiter.rev_limit", units::rpm(8000.0));
    fixture.modes.add(sport);

    powertrain::BootstrapInputs inputs;
    inputs.unit = makeUnit();
    inputs.defaultMode = "sport";
    inputs.parameterOverrides.push_back({ "ecu.limiter.rev_limit", units::rpm(7600.0) });

    const powertrain::BootstrapResult result =
        powertrain::installPowertrain(inputs, fixture.context());

    EXPECT_EQ(result.defaultModeIndex, 0);
    EXPECT_NEAR(value(fixture.registry, "ecu.limiter.rev_limit"), units::rpm(8000.0), 1e-6);

    delete inputs.unit;
}

TEST(PowertrainBootstrapTests, AnUnknownDefaultModeLeavesTheOverrideAlone) {
    Fixture fixture;

    powertrain::BootstrapInputs inputs;
    inputs.unit = makeUnit();
    inputs.defaultMode = "track";
    inputs.parameterOverrides.push_back({ "ecu.limiter.rev_limit", units::rpm(7600.0) });

    const powertrain::BootstrapResult result =
        powertrain::installPowertrain(inputs, fixture.context());

    EXPECT_EQ(result.defaultModeIndex, -1);
    EXPECT_NEAR(value(fixture.registry, "ecu.limiter.rev_limit"), units::rpm(7600.0), 1e-6);

    delete inputs.unit;
}

TEST(PowertrainBootstrapTests, WithoutAControllerNothingIsInstalled) {
    Fixture fixture;

    powertrain::BootstrapInputs inputs;

    const powertrain::BootstrapResult result =
        powertrain::installPowertrain(inputs, fixture.context());

    EXPECT_EQ(result.controller, nullptr);
    EXPECT_EQ(fixture.system.getController(), nullptr);
    EXPECT_FALSE(result.adaptationAttached);
}

TEST(PowertrainBootstrapTests, AdaptiveGrantsLandAfterTheRegistryIsBuilt) {
    Fixture fixture;

    powertrain::BootstrapInputs inputs;
    inputs.unit = makeUnit();

    powertrain::AdaptiveOverride grant;
    grant.path = "tcu.shift.min_gear_time";
    grant.adaptive = true;
    grant.adaptMin = 0.1;
    grant.adaptMax = 2.0;
    inputs.adaptiveOverrides.push_back(grant);

    powertrain::installPowertrain(inputs, fixture.context());

    ASSERT_TRUE(fixture.registry.isAdaptive("tcu.shift.min_gear_time"));
    ASSERT_TRUE(fixture.registry.adapt("tcu.shift.min_gear_time", 0.2));

    EXPECT_NEAR(
        inputs.unit->getTransmissionControlUnit().getParameters().minGearTime,
        1.0,
        1e-9);

    ASSERT_TRUE(fixture.registry.adapt("tcu.shift.min_gear_time", 10.0));
    EXPECT_NEAR(
        inputs.unit->getTransmissionControlUnit().getParameters().minGearTime,
        2.0,
        1e-9);

    delete inputs.unit;
}

TEST(PowertrainBootstrapTests, WithoutAGrantTheRegistryStillRefuses) {
    Fixture fixture;

    powertrain::BootstrapInputs inputs;
    inputs.unit = makeUnit();

    powertrain::installPowertrain(inputs, fixture.context());

    EXPECT_FALSE(fixture.registry.isAdaptive("tcu.shift.min_gear_time"));
    EXPECT_FALSE(fixture.registry.adapt("tcu.shift.min_gear_time", 0.2));

    delete inputs.unit;
}

TEST(PowertrainBootstrapTests, TheThreeControlModesAreNamed) {
    powertrain::PowertrainUnit unit;
    powertrain::ScriptedControlUnit program;

    const auto scriptOnly = powertrain::selectControllers(nullptr, &program);
    EXPECT_EQ(scriptOnly.mode, powertrain::ControlMode::ScriptOnly);
    EXPECT_EQ(scriptOnly.primary, &program);
    EXPECT_EQ(scriptOnly.overlay, nullptr);

    const auto units = powertrain::selectControllers(&unit, nullptr);
    EXPECT_EQ(units.mode, powertrain::ControlMode::ControlUnits);
    EXPECT_EQ(units.primary, &unit);
    EXPECT_EQ(units.overlay, nullptr);

    const auto overlay = powertrain::selectControllers(&unit, &program);
    EXPECT_EQ(overlay.mode, powertrain::ControlMode::ScriptOverlay);
    EXPECT_EQ(overlay.primary, &unit);
    EXPECT_EQ(overlay.overlay, &program);
}
