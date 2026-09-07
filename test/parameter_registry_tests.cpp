#include <gtest/gtest.h>

#include "../include/config/parameter_registry.h"
#include "../include/control/map_2d.h"
#include "../include/config/drive_mode.h"

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

TEST(ParameterRegistryTests, AScriptValueOutsideTheRangeWidensTheRange) {
    config::ParameterRegistry registry;
    double value = 0.0;

    registry.registerScalar(scalar("ecu.idle.kp", 0.0, 1.0, 5.0), &value);

    EXPECT_NEAR(value, 5.0, 1e-12)
        << "registering silently changed what the script asked for";

    ASSERT_EQ(registry.getCount(), 1);
    EXPECT_NEAR(registry.getDescriptor(0).maxValue, 5.0, 1e-12);

    ASSERT_TRUE(registry.set("ecu.idle.kp", 4.0));
    EXPECT_NEAR(value, 4.0, 1e-12);
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

namespace {
    void fillMap(control::Map2d *map, int xCount, int yCount, double value) {
        map->initialize(xCount, yCount, value);

        for (int i = 0; i < xCount; ++i) map->setXAxis(i, static_cast<double>(i));
        for (int j = 0; j < yCount; ++j) map->setYAxis(j, static_cast<double>(j));
    }

    config::ParameterDescriptor describeMap(const char *path) {
        config::ParameterDescriptor d;
        d.path = path;
        d.minValue = 0.0;
        d.maxValue = 100.0;
        d.defaultValue = 0.0;

        return d;
    }
}

TEST(MapCellTests, ACellIsReachableByPath) {
    config::ParameterRegistry registry;
    control::Map2d map;
    fillMap(&map, 4, 3, 7.0);

    registry.registerMap(describeMap("tcu.upshift_map"), &map);

    EXPECT_TRUE(registry.contains("tcu.upshift_map[2][1]"));

    double value = 0.0;
    ASSERT_TRUE(registry.get("tcu.upshift_map[2][1]", &value));
    EXPECT_NEAR(value, 7.0, 1e-12);

    ASSERT_TRUE(registry.set("tcu.upshift_map[2][1]", 42.0));
    EXPECT_NEAR(map.getValue(2, 1), 42.0, 1e-12);
    EXPECT_NEAR(map.getValue(1, 1), 7.0, 1e-12) << "a neighbour was written too";
}

TEST(MapCellTests, OutOfRangeAndMalformedPathsAreRejected) {
    config::ParameterRegistry registry;
    control::Map2d map;
    fillMap(&map, 4, 3, 7.0);

    registry.registerMap(describeMap("tcu.upshift_map"), &map);

    EXPECT_FALSE(registry.set("tcu.upshift_map[4][0]", 1.0));
    EXPECT_FALSE(registry.set("tcu.upshift_map[0][3]", 1.0));
    EXPECT_FALSE(registry.set("tcu.upshift_map[-1][0]", 1.0));
    EXPECT_FALSE(registry.set("tcu.upshift_map[0]", 1.0));
    EXPECT_FALSE(registry.set("tcu.upshift_map[a][0]", 1.0));
    EXPECT_FALSE(registry.set("tcu.missing_map[0][0]", 1.0));
    EXPECT_FALSE(registry.contains("tcu.upshift_map[9][9]"));
}

TEST(MapCellTests, TheWholeMapItselfStaysUnwritable) {
    config::ParameterRegistry registry;
    control::Map2d map;
    fillMap(&map, 4, 3, 7.0);

    registry.registerMap(describeMap("tcu.upshift_map"), &map);

    EXPECT_FALSE(registry.set("tcu.upshift_map", 1.0));

    double value = 0.0;
    EXPECT_FALSE(registry.get("tcu.upshift_map", &value));
}

TEST(MapCellTests, ADriveModeCarriesAMapAndGivesItBack) {
    config::ParameterRegistry registry;
    control::Map2d map;
    fillMap(&map, 4, 3, 7.0);

    registry.registerMap(describeMap("tcu.upshift_map"), &map);

    config::DriveMode sport("sport");
    sport.set("tcu.upshift_map[1][0]", 30.0);
    sport.set("tcu.upshift_map[2][0]", 40.0);

    config::DriveMode comfort("comfort");
    comfort.set("tcu.upshift_map[1][0]", 12.0);

    config::DriveModeSet modes;
    modes.add(sport);
    modes.add(comfort);

    ASSERT_TRUE(modes.select("sport", &registry));
    EXPECT_NEAR(map.getValue(1, 0), 30.0, 1e-12);
    EXPECT_NEAR(map.getValue(2, 0), 40.0, 1e-12);

    ASSERT_TRUE(modes.select("comfort", &registry));
    EXPECT_NEAR(map.getValue(1, 0), 12.0, 1e-12);
    EXPECT_NEAR(map.getValue(2, 0), 7.0, 1e-12)
        << "the previous mode's cell was not restored";
}

TEST(AdaptiveGateTests, TickingTheBoxDoesNotCollapseTheParameter) {
    config::ParameterRegistry registry;
    double value = 5.0;

    registry.registerScalar(
        config::describeScalar("ecu.knob", 0.0, 10.0, 5.0, ""), &value);

    ASSERT_TRUE(registry.setAdaptive("ecu.knob", true));
    ASSERT_TRUE(registry.adapt("ecu.knob", 0.5));

    EXPECT_NEAR(value, 5.5, 1e-12);
}

TEST(AdaptiveGateTests, TheFallbackRespectsTheDeclaredRange) {
    config::ParameterRegistry registry;
    double value = 5.0;

    registry.registerScalar(
        config::describeScalar("ecu.knob", 0.0, 10.0, 5.0, ""), &value);

    ASSERT_TRUE(registry.setAdaptive("ecu.knob", true));
    ASSERT_TRUE(registry.adapt("ecu.knob", 100.0));

    EXPECT_NEAR(value, 10.0, 1e-12);
}

TEST(AdaptiveGateTests, ExplicitBoundsStillWin) {
    config::ParameterRegistry registry;
    double value = 5.0;

    registry.registerScalar(
        config::describeScalar("ecu.knob", 0.0, 10.0, 5.0, ""), &value);

    ASSERT_TRUE(registry.setAdaptive("ecu.knob", true, 4.0, 6.0));
    ASSERT_TRUE(registry.adapt("ecu.knob", 100.0));

    EXPECT_NEAR(value, 6.0, 1e-12);
}

TEST(ParameterRegistryTests, ResetRestoresTheAuthoredMapCells) {
    config::ParameterRegistry registry;

    control::Map2d map;
    map.initialize(2, 2, 0.0);
    map.setXAxis(0, 0.0);
    map.setXAxis(1, 1.0);
    map.setYAxis(0, 0.0);
    map.setYAxis(1, 1.0);
    map.setValue(0, 0, 0.25);
    map.setValue(1, 1, 0.75);

    config::ParameterDescriptor descriptor =
        config::describeScalar("ecu.throttle_map", 0.0, 1.0, 0.0, "");
    descriptor.adaptive = true;
    descriptor.adaptMax = 1.0;

    ASSERT_TRUE(registry.registerMap(descriptor, &map));
    ASSERT_TRUE(registry.accumulate("ecu.throttle_map", 0.0, 0.0, 0.5));

    ASSERT_GT(map.getValue(0, 0), 0.25);

    registry.resetToDefaults();

    EXPECT_NEAR(map.getValue(0, 0), 0.25, 1e-12)
        << "reset left the learned cell in place";
    EXPECT_NEAR(map.getValue(1, 1), 0.75, 1e-12);
}

TEST(ParameterRegistryTests, TheDisplayRangeIsSeparateFromTheValidRange) {
    config::ParameterRegistry registry;
    double value = 2.0;

    config::ParameterDescriptor descriptor =
        config::describeScalar("program.gain.gain", -1e9, 1e9, 2.0, "");
    descriptor.displayMin = -8.0;
    descriptor.displayMax = 8.0;

    ASSERT_TRUE(registry.registerScalar(descriptor, &value));

    std::ostringstream out;
    registry.serializeJson(out);
    const std::string json = out.str();

    EXPECT_NE(json.find("\"display_min\":-8"), std::string::npos) << json;
    EXPECT_NE(json.find("\"display_max\":8"), std::string::npos) << json;

    ASSERT_TRUE(registry.set("program.gain.gain", 5000.0));
    EXPECT_NEAR(value, 5000.0, 1e-12)
        << "the display range clamped a value it has no business clamping";
}
