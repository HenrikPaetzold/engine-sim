#include <gtest/gtest.h>

#include "../include/config/parameter_registry.h"
#include "../include/control/map_2d.h"

#include <sstream>

namespace {
    config::ParameterDescriptor scalar(
        const char *path,
        double min,
        double max,
        double defaultValue)
    {
        config::ParameterDescriptor d;
        d.path = path;
        d.minValue = min;
        d.maxValue = max;
        d.defaultValue = defaultValue;

        return d;
    }
}

TEST(ParameterRegistryTests, RegistrationAppliesTheDefault) {
    config::ParameterRegistry registry;
    double value = 123.0;

    ASSERT_TRUE(registry.registerScalar(scalar("ecu.idle.kp", 0.0, 10.0, 2.5), &value));
    EXPECT_NEAR(value, 2.5, 1e-12);
}

TEST(ParameterRegistryTests, DefaultOutsideRangeIsClamped) {
    config::ParameterRegistry registry;
    double value = 0.0;

    registry.registerScalar(scalar("ecu.idle.kp", 0.0, 1.0, 5.0), &value);

    EXPECT_NEAR(value, 1.0, 1e-12);
}

TEST(ParameterRegistryTests, SetWritesThroughAndClamps) {
    config::ParameterRegistry registry;
    double value = 0.0;
    registry.registerScalar(scalar("ecu.idle.kp", 0.0, 10.0, 1.0), &value);

    ASSERT_TRUE(registry.set("ecu.idle.kp", 4.0));
    EXPECT_NEAR(value, 4.0, 1e-12);

    ASSERT_TRUE(registry.set("ecu.idle.kp", 1000.0));
    EXPECT_NEAR(value, 10.0, 1e-12);

    double readBack = 0.0;
    ASSERT_TRUE(registry.get("ecu.idle.kp", &readBack));
    EXPECT_NEAR(readBack, 10.0, 1e-12);
}

TEST(ParameterRegistryTests, UnknownPathIsRejected) {
    config::ParameterRegistry registry;
    double value = 0.0;

    EXPECT_FALSE(registry.set("does.not.exist", 1.0));
    EXPECT_FALSE(registry.get("does.not.exist", &value));
    EXPECT_FALSE(registry.contains("does.not.exist"));
}

TEST(ParameterRegistryTests, DuplicatePathIsRejected) {
    config::ParameterRegistry registry;
    double a = 0.0;
    double b = 0.0;

    ASSERT_TRUE(registry.registerScalar(scalar("ecu.idle.kp", 0.0, 1.0, 0.5), &a));
    EXPECT_FALSE(registry.registerScalar(scalar("ecu.idle.kp", 0.0, 1.0, 0.5), &b));
    EXPECT_EQ(registry.getCount(), 1);
}

TEST(ParameterRegistryTests, IntegerAndBooleanRoundTrip) {
    config::ParameterRegistry registry;
    int gear = 0;
    bool enabled = false;

    config::ParameterDescriptor gearDescriptor = scalar("tcu.max_gear", 0.0, 8.0, 6.0);
    config::ParameterDescriptor enabledDescriptor = scalar("tcu.enabled", 0.0, 1.0, 1.0);

    registry.registerInteger(gearDescriptor, &gear);
    registry.registerBoolean(enabledDescriptor, &enabled);

    EXPECT_EQ(gear, 6);
    EXPECT_TRUE(enabled);

    registry.set("tcu.max_gear", 3.4);
    registry.set("tcu.enabled", 0.0);

    EXPECT_EQ(gear, 3);
    EXPECT_FALSE(enabled);
}

TEST(ParameterRegistryTests, AdaptionOnlyTouchesAdaptiveParameters) {
    config::ParameterRegistry registry;
    double fixed = 0.0;
    double learned = 0.0;

    registry.registerScalar(scalar("ecu.fixed", 0.0, 10.0, 1.0), &fixed);

    config::ParameterDescriptor adaptive = scalar("ecu.learned", -10.0, 10.0, 0.0);
    adaptive.adaptive = true;
    adaptive.adaptMin = -0.5;
    adaptive.adaptMax = 0.5;
    registry.registerScalar(adaptive, &learned);

    EXPECT_FALSE(registry.adapt("ecu.fixed", 1.0));
    EXPECT_NEAR(fixed, 1.0, 1e-12);

    ASSERT_TRUE(registry.adapt("ecu.learned", 0.2));
    EXPECT_NEAR(learned, 0.2, 1e-12);
}

TEST(ParameterRegistryTests, AdaptionIsBoundedByItsOwnLimits) {
    config::ParameterRegistry registry;
    double learned = 0.0;

    config::ParameterDescriptor adaptive = scalar("ecu.learned", -100.0, 100.0, 0.0);
    adaptive.adaptive = true;
    adaptive.adaptMin = -0.5;
    adaptive.adaptMax = 0.5;
    registry.registerScalar(adaptive, &learned);

    for (int i = 0; i < 1000; ++i) registry.adapt("ecu.learned", 1.0);
    EXPECT_NEAR(learned, 0.5, 1e-12);

    for (int i = 0; i < 1000; ++i) registry.adapt("ecu.learned", -1.0);
    EXPECT_NEAR(learned, -0.5, 1e-12);
}

TEST(ParameterRegistryTests, ResetToDefaultsRestoresEveryValue) {
    config::ParameterRegistry registry;
    double a = 0.0;
    int b = 0;

    registry.registerScalar(scalar("a", 0.0, 10.0, 3.0), &a);
    registry.registerInteger(scalar("b", 0.0, 10.0, 4.0), &b);

    registry.set("a", 9.0);
    registry.set("b", 9.0);

    registry.resetToDefaults();

    EXPECT_NEAR(a, 3.0, 1e-12);
    EXPECT_EQ(b, 4);
}

TEST(ParameterRegistryTests, SchemaContainsPathsValuesAndLimits) {
    config::ParameterRegistry registry;
    double value = 0.0;

    config::ParameterDescriptor d = scalar("ecu.idle.kp", 0.0, 10.0, 2.0);
    d.unit = "";
    registry.registerScalar(d, &value);

    std::ostringstream out;
    registry.serializeJson(out);
    const std::string json = out.str();

    EXPECT_NE(json.find("\"ecu.idle.kp\""), std::string::npos);
    EXPECT_NE(json.find("\"value\":2"), std::string::npos);
    EXPECT_NE(json.find("\"max\":10"), std::string::npos);
    EXPECT_NE(json.find("\"adaptive\":false"), std::string::npos);
}

TEST(ParameterRegistryTests, MapIsSerializedWithAxesAndValues) {
    config::ParameterRegistry registry;
    control::Map2d map;
    map.initialize(2, 2, 0.0);
    map.setXAxis(0, 1000.0);
    map.setXAxis(1, 6000.0);
    map.setValue(1, 1, 42.0);

    config::ParameterDescriptor d = scalar("ecu.throttle_map", 0.0, 1.0, 0.0);
    registry.registerMap(d, &map);

    std::ostringstream out;
    registry.serializeJson(out);
    const std::string json = out.str();

    EXPECT_NE(json.find("\"xAxis\":[1000,6000]"), std::string::npos);
    EXPECT_NE(json.find("42"), std::string::npos);
}

TEST(ParameterRegistryTests, ExportEmitsOnlyLearnedValues) {
    config::ParameterRegistry registry;
    double fixed = 1.0;
    double learned = 0.0;

    registry.registerScalar(scalar("ecu.fixed", 0.0, 10.0, 1.0), &fixed);

    config::ParameterDescriptor adaptive = scalar("ecu.learned", -10.0, 10.0, 0.0);
    adaptive.adaptive = true;
    adaptive.adaptMin = -10.0;
    adaptive.adaptMax = 10.0;
    registry.registerScalar(adaptive, &learned);
    registry.adapt("ecu.learned", 0.25);

    std::ostringstream out;
    registry.exportScript(out);
    const std::string script = out.str();

    EXPECT_NE(script.find("set_parameter(\"ecu.learned\", 0.25)"), std::string::npos);
    EXPECT_EQ(script.find("ecu.fixed"), std::string::npos);
}
