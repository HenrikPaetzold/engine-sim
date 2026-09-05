#include <gtest/gtest.h>

#include "../include/config/config_server.h"
#include "../include/config/parameter_registry.h"
#include "../include/config/drive_mode.h"
#include "../include/control/map_2d.h"
#include "../include/config/channel_recorder.h"

#include "../dependencies/cpp-httplib/httplib.h"

#include <string>

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

    class ServerFixture : public ::testing::Test {
        protected:
            void SetUp() override {
                m_registry.registerScalar(scalar("ecu.limiter.rev_limit", 0.0, 1000.0, 700.0), &m_revLimit);
                m_registry.registerScalar(scalar("environment.road_grade", -0.3, 0.3, 0.0), &m_grade);

                config::ParameterDescriptor learned = scalar("ecu.idle.trim", -1.0, 1.0, 0.0);
                learned.adaptive = true;
                learned.adaptMin = -1.0;
                learned.adaptMax = 1.0;
                m_registry.registerScalar(learned, &m_trim);

                m_map.initialize(3, 2, 5.0);
                for (int i = 0; i < 3; ++i) m_map.setXAxis(i, i);
                for (int j = 0; j < 2; ++j) m_map.setYAxis(j, j);

                config::ParameterDescriptor table = scalar("tcu.upshift_map", 0.0, 100.0, 0.0);
                table.type = config::ParameterType::Map;
                m_registry.registerMap(table, &m_map);

                config::DriveMode sport("sport");
                sport.set("ecu.limiter.rev_limit", 900.0);
                m_modes.add(sport);

                config::DriveMode eco("eco");
                eco.set("ecu.limiter.rev_limit", 400.0);
                m_modes.add(eco);

                config::ConfigServer::Parameters params;
                params.port = 0;
                params.uiPath = "does-not-exist.html";

                m_server.initialize(params, &m_registry, &m_modes);
                ASSERT_TRUE(m_server.start());
                ASSERT_GT(m_server.getBoundPort(), 0);
            }

            void TearDown() override {
                m_server.stop();
            }

            httplib::Client client() {
                httplib::Client c("127.0.0.1", m_server.getBoundPort());
                c.set_connection_timeout(2, 0);
                c.set_read_timeout(2, 0);

                return c;
            }

            control::Map2d m_map;
            config::ParameterRegistry m_registry;
            config::DriveModeSet m_modes;
            config::ConfigServer m_server;

            double m_revLimit = 0.0;
            double m_grade = 0.0;
            double m_trim = 0.0;
    };
}

TEST_F(ServerFixture, SchemaListsEveryParameterAndMode) {
    auto response = client().Get("/api/schema");

    ASSERT_TRUE(response);
    ASSERT_EQ(response->status, 200);

    EXPECT_NE(response->body.find("ecu.limiter.rev_limit"), std::string::npos);
    EXPECT_NE(response->body.find("environment.road_grade"), std::string::npos);
    EXPECT_NE(response->body.find("\"sport\""), std::string::npos);
    EXPECT_NE(response->body.find("\"eco\""), std::string::npos);
    EXPECT_NE(response->body.find("\"adaptive\":true"), std::string::npos);
}

TEST_F(ServerFixture, StateReportsCurrentValues) {
    m_server.publish(config::TelemetrySample());

    auto response = client().Get("/api/state");

    ASSERT_TRUE(response);
    EXPECT_NE(response->body.find("\"ecu.limiter.rev_limit\":700"), std::string::npos);
    EXPECT_NE(response->body.find("\"telemetry\""), std::string::npos);
}

TEST_F(ServerFixture, TelemetryReachesTheClient) {
    config::TelemetrySample sample;
    sample.engineRpm = 3210.0;
    sample.gear = 2;
    sample.engineState = "Running";
    sample.shiftState = "Idle";
    m_server.publish(sample);

    auto response = client().Get("/api/state");

    ASSERT_TRUE(response);
    EXPECT_NE(response->body.find("\"engineRpm\":3210"), std::string::npos);
    EXPECT_NE(response->body.find("\"gear\":2"), std::string::npos);
    EXPECT_NE(response->body.find("\"engineState\":\"Running\""), std::string::npos);
}

TEST_F(ServerFixture, SetIsQueuedAndNotAppliedUntilTheSimulationAsks) {
    auto response = client().Post(
        "/api/set",
        "{\"path\":\"ecu.limiter.rev_limit\",\"value\":850.0}",
        "application/json");

    ASSERT_TRUE(response);
    ASSERT_EQ(response->status, 200);

    EXPECT_NEAR(m_revLimit, 700.0, 1e-12);

    EXPECT_EQ(m_server.applyPendingCommands(), 1);
    EXPECT_NEAR(m_revLimit, 850.0, 1e-12);
}

TEST_F(ServerFixture, SetIsClampedByTheRegistry) {
    client().Post(
        "/api/set",
        "{\"path\":\"environment.road_grade\",\"value\":5.0}",
        "application/json");

    m_server.applyPendingCommands();

    EXPECT_NEAR(m_grade, 0.3, 1e-12);
}

TEST_F(ServerFixture, RoadGradeCanBeDrivenFromTheBrowser) {
    client().Post(
        "/api/set",
        "{\"path\":\"environment.road_grade\",\"value\":0.08}",
        "application/json");

    m_server.applyPendingCommands();

    EXPECT_NEAR(m_grade, 0.08, 1e-12);
}

TEST_F(ServerFixture, MalformedRequestsAreRejected) {
    auto response = client().Post("/api/set", "{\"nonsense\":1}", "application/json");

    ASSERT_TRUE(response);
    EXPECT_EQ(response->status, 400);
    EXPECT_EQ(m_server.applyPendingCommands(), 0);
}

TEST_F(ServerFixture, UnknownPathIsAcceptedButChangesNothing) {
    client().Post(
        "/api/set",
        "{\"path\":\"not.a.parameter\",\"value\":1.0}",
        "application/json");

    EXPECT_EQ(m_server.applyPendingCommands(), 0);
}

TEST_F(ServerFixture, ModeSelectionGoesThroughTheQueue) {
    auto response = client().Post("/api/mode", "{\"index\":0}", "application/json");

    ASSERT_TRUE(response);
    ASSERT_EQ(response->status, 200);

    EXPECT_NEAR(m_revLimit, 700.0, 1e-12);
    EXPECT_EQ(m_server.applyPendingCommands(), 1);
    EXPECT_NEAR(m_revLimit, 900.0, 1e-12);

    client().Post("/api/mode", "{\"index\":1}", "application/json");
    m_server.applyPendingCommands();
    EXPECT_NEAR(m_revLimit, 400.0, 1e-12);
}

TEST_F(ServerFixture, ResetRestoresTheDefaults) {
    client().Post("/api/set", "{\"path\":\"ecu.limiter.rev_limit\",\"value\":999.0}", "application/json");
    m_server.applyPendingCommands();
    ASSERT_NEAR(m_revLimit, 999.0, 1e-12);

    client().Post("/api/reset", "{}", "application/json");
    m_server.applyPendingCommands();

    EXPECT_NEAR(m_revLimit, 700.0, 1e-12);
}

TEST_F(ServerFixture, ExportOffersOnlyLearnedValues) {
    m_trim = 0.125;
    m_server.publish(config::TelemetrySample());

    auto response = client().Get("/api/export");

    ASSERT_TRUE(response);
    EXPECT_NE(response->body.find("set_parameter(\"ecu.idle.trim\", 0.125)"), std::string::npos);
    EXPECT_EQ(response->body.find("rev_limit"), std::string::npos);
}

TEST_F(ServerFixture, ExportedValuesCanBeReadBackIn) {
    client().Post("/api/set", "{\"path\":\"ecu.idle.trim\",\"value\":0.4}", "application/json");
    m_server.applyPendingCommands();

    m_server.publish(config::TelemetrySample());
    const std::string exported = m_server.exportScript();

    client().Post("/api/reset", "{}", "application/json");
    m_server.applyPendingCommands();
    ASSERT_NEAR(m_trim, 0.0, 1e-12);

    const size_t open = exported.find(", ");
    const size_t close = exported.find(')', open);
    ASSERT_NE(open, std::string::npos);
    ASSERT_NE(close, std::string::npos);

    const double value = std::stod(exported.substr(open + 2, close - open - 2));
    ASSERT_TRUE(m_registry.set("ecu.idle.trim", value));

    EXPECT_NEAR(m_trim, 0.4, 1e-12);
}

TEST_F(ServerFixture, MissingUiFileReportsNotFound) {
    auto response = client().Get("/");

    ASSERT_TRUE(response);
    EXPECT_EQ(response->status, 404);
}

TEST_F(ServerFixture, ServerStopsCleanly) {
    ASSERT_TRUE(m_server.isRunning());
    m_server.stop();
    EXPECT_FALSE(m_server.isRunning());

    m_server.stop();
    EXPECT_FALSE(m_server.isRunning());
}

TEST_F(ServerFixture, TheStateCarriesTheMapValues) {
    m_server.publish(config::TelemetrySample());

    auto response = client().Get("/api/state");
    ASSERT_TRUE(response);

    EXPECT_NE(response->body.find("\"maps\""), std::string::npos)
        << "the live state has no map section";
    EXPECT_NE(response->body.find("\"tcu.upshift_map\":[5,5,5,5,5,5]"), std::string::npos);
}

TEST_F(ServerFixture, AWrittenCellComesBackInTheLiveState) {
    auto post = client().Post(
        "/api/set",
        "{\"path\":\"tcu.upshift_map[1][0]\",\"value\":42}",
        "application/json");

    ASSERT_TRUE(post);
    ASSERT_EQ(post->status, 200);

    EXPECT_EQ(m_server.applyPendingCommands(), 1);

    m_server.publish(config::TelemetrySample());

    auto response = client().Get("/api/state");
    ASSERT_TRUE(response);

    EXPECT_NE(response->body.find("\"tcu.upshift_map\":[5,42,5,5,5,5]"), std::string::npos)
        << "the written cell did not reach the live state";
}

TEST_F(ServerFixture, TheStateCarriesTheAdaptiveFlags) {
    m_server.publish(config::TelemetrySample());

    auto first = client().Get("/api/state");
    ASSERT_TRUE(first);
    EXPECT_NE(first->body.find("\"ecu.idle.trim\":true"), std::string::npos);
    EXPECT_NE(first->body.find("\"ecu.limiter.rev_limit\":false"), std::string::npos);

    auto post = client().Post(
        "/api/adaptive",
        "{\"path\":\"ecu.limiter.rev_limit\",\"value\":1}",
        "application/json");

    ASSERT_TRUE(post);
    EXPECT_EQ(m_server.applyPendingCommands(), 1);

    m_server.publish(config::TelemetrySample());

    auto second = client().Get("/api/state");
    ASSERT_TRUE(second);
    EXPECT_NE(second->body.find("\"ecu.limiter.rev_limit\":true"), std::string::npos)
        << "the adaptive flag did not follow";
}

TEST_F(ServerFixture, TheScopeChannelsAreSelectableFromTheBrowser) {
    config::ChannelRecorder scope;
    scope.initialize(1.0);
    m_server.setScope(&scope);

    auto post = client().Post(
        "/api/scope",
        "{\"channels\":[\"engine_rpm\",\"clutch_pressure\"],\"window\":2.5}",
        "application/json");

    ASSERT_TRUE(post);
    ASSERT_EQ(post->status, 200);
    EXPECT_EQ(m_server.applyPendingCommands(), 2);

    ASSERT_EQ(scope.getChannelCount(), 2);
    EXPECT_EQ(scope.getChannelName(0), "engine_rpm");
    EXPECT_EQ(scope.getChannelName(1), "clutch_pressure");
    EXPECT_NEAR(scope.getWindow(), 2.5, 1e-9);
}

TEST_F(ServerFixture, TheScopeIsPublishedAndReadable) {
    config::ChannelTable table;
    table.set("engine_rpm", 2500.0);

    config::ChannelRecorder scope;
    scope.initialize(1.0);
    scope.select({ "engine_rpm" });

    for (int i = 0; i < 500; ++i) scope.update(1e-3, i * 1e-3, table);

    m_server.publishScope(scope, table);

    auto response = client().Get("/api/scope");
    ASSERT_TRUE(response);
    EXPECT_NE(response->body.find("\"name\":\"engine_rpm\""), std::string::npos);
    EXPECT_NE(response->body.find("2500"), std::string::npos);

    auto names = client().Get("/api/channels");
    ASSERT_TRUE(names);
    EXPECT_NE(names->body.find("engine_rpm"), std::string::npos);
}

TEST_F(ServerFixture, ATriggeredScopeIsArmedFromTheBrowser) {
    config::ChannelRecorder scope;
    scope.initialize(1.0);
    scope.select({ "engine_rpm" });
    scope.setMode(config::ChannelRecorder::Mode::Triggered);
    m_server.setScope(&scope);

    ASSERT_FALSE(scope.isArmed());

    auto post = client().Post("/api/scope", "{\"arm\":1}", "application/json");
    ASSERT_TRUE(post);
    EXPECT_EQ(m_server.applyPendingCommands(), 1);

    EXPECT_TRUE(scope.isArmed());
}
