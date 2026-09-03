#include <gtest/gtest.h>

#include "../include/config/drive_mode.h"
#include "../include/config/parameter_registry.h"

namespace {
    config::ParameterDescriptor scalar(const char *path, double defaultValue) {
        config::ParameterDescriptor d;
        d.path = path;
        d.minValue = 0.0;
        d.maxValue = 100.0;
        d.defaultValue = defaultValue;

        return d;
    }
}

TEST(DriveModeTests, AppliesOnlyItsOwnOverrides) {
    config::ParameterRegistry registry;
    double a = 0.0;
    double b = 0.0;

    registry.registerScalar(scalar("ecu.a", 1.0), &a);
    registry.registerScalar(scalar("ecu.b", 2.0), &b);

    config::DriveMode sport("sport");
    sport.set("ecu.a", 9.0);

    config::DriveModeSet modes;
    modes.add(sport);

    ASSERT_TRUE(modes.select("sport", &registry));

    EXPECT_NEAR(a, 9.0, 1e-12);
    EXPECT_NEAR(b, 2.0, 1e-12);
}

TEST(DriveModeTests, SwitchingRestoresWhatTheOtherModeChanged) {
    config::ParameterRegistry registry;
    double a = 0.0;
    double b = 0.0;

    registry.registerScalar(scalar("ecu.a", 1.0), &a);
    registry.registerScalar(scalar("ecu.b", 2.0), &b);

    config::DriveMode sport("sport");
    sport.set("ecu.a", 9.0);

    config::DriveMode comfort("comfort");
    comfort.set("ecu.b", 8.0);

    config::DriveModeSet modes;
    modes.add(sport);
    modes.add(comfort);

    ASSERT_TRUE(modes.select("sport", &registry));
    EXPECT_NEAR(a, 9.0, 1e-12);
    EXPECT_NEAR(b, 2.0, 1e-12);

    ASSERT_TRUE(modes.select("comfort", &registry));
    EXPECT_NEAR(a, 1.0, 1e-12);
    EXPECT_NEAR(b, 8.0, 1e-12);
}

TEST(DriveModeTests, AnyNumberOfFreelyNamedModes) {
    config::ParameterRegistry registry;
    double value = 0.0;
    registry.registerScalar(scalar("ecu.value", 0.0), &value);

    config::DriveModeSet modes;
    for (int i = 0; i < 12; ++i) {
        config::DriveMode mode("mode_" + std::to_string(i));
        mode.set("ecu.value", static_cast<double>(i));
        modes.add(mode);
    }

    EXPECT_EQ(modes.getCount(), 12);

    ASSERT_TRUE(modes.select("mode_7", &registry));
    EXPECT_NEAR(value, 7.0, 1e-12);
    EXPECT_EQ(modes.getSelected(), 7);
}

TEST(DriveModeTests, UnknownModeIsRejected) {
    config::ParameterRegistry registry;
    config::DriveModeSet modes;

    EXPECT_FALSE(modes.select("nope", &registry));
    EXPECT_FALSE(modes.select(3, &registry));
}

TEST(DriveModeTests, UnknownPathsAreIgnored) {
    config::ParameterRegistry registry;
    double value = 0.0;
    registry.registerScalar(scalar("ecu.value", 1.0), &value);

    config::DriveMode mode("odd");
    mode.set("ecu.value", 5.0);
    mode.set("does.not.exist", 42.0);

    config::DriveModeSet modes;
    modes.add(mode);

    ASSERT_TRUE(modes.select(0, &registry));
    EXPECT_NEAR(value, 5.0, 1e-12);
}

TEST(DriveModeTests, RepeatedPathKeepsTheLastValue) {
    config::DriveMode mode("x");
    mode.set("a", 1.0);
    mode.set("a", 2.0);

    ASSERT_EQ(mode.getOverrideCount(), 1);
    EXPECT_NEAR(mode.getOverride(0).value, 2.0, 1e-12);
}

TEST(DriveModeTests, BaselineIsTheValueBeforeTheFirstSelection) {
    config::ParameterRegistry registry;
    double value = 0.0;
    registry.registerScalar(scalar("ecu.value", 1.0), &value);

    registry.set("ecu.value", 4.0);

    config::DriveMode mode("m");
    mode.set("ecu.value", 9.0);

    config::DriveModeSet modes;
    modes.add(mode);

    ASSERT_TRUE(modes.select(0, &registry));
    EXPECT_NEAR(value, 9.0, 1e-12);

    modes.restoreBaseline(&registry);
    EXPECT_NEAR(value, 4.0, 1e-12);
}
