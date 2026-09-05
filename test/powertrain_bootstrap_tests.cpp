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
