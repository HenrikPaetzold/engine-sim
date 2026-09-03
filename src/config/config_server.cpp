#include "../../include/config/config_server.h"

#include "../../include/config/parameter_registry.h"
#include "../../include/config/drive_mode.h"

#include "../../dependencies/cpp-httplib/httplib.h"

#include <fstream>
#include <sstream>

namespace {
    httplib::Server *serverOf(void *handle) {
        return static_cast<httplib::Server *>(handle);
    }

    std::string jsonString(const std::string &value) {
        std::string out = "\"";
        for (char c : value) {
            if (c == '"' || c == '\\') { out += '\\'; out += c; }
            else if (c == '\n') out += "\\n";
            else out += c;
        }

        return out + "\"";
    }

    bool extractNumber(const std::string &body, const std::string &key, double *value) {
        const std::string needle = "\"" + key + "\"";
        const size_t at = body.find(needle);
        if (at == std::string::npos) return false;

        const size_t colon = body.find(':', at + needle.size());
        if (colon == std::string::npos) return false;

        try {
            *value = std::stod(body.substr(colon + 1));
        }
        catch (...) {
            return false;
        }

        return true;
    }

    bool extractString(const std::string &body, const std::string &key, std::string *value) {
        const std::string needle = "\"" + key + "\"";
        const size_t at = body.find(needle);
        if (at == std::string::npos) return false;

        const size_t colon = body.find(':', at + needle.size());
        if (colon == std::string::npos) return false;

        const size_t open = body.find('"', colon);
        if (open == std::string::npos) return false;

        std::string out;
        for (size_t i = open + 1; i < body.size(); ++i) {
            if (body[i] == '\\' && i + 1 < body.size()) {
                out += body[i + 1];
                ++i;
                continue;
            }
            if (body[i] == '"') {
                *value = out;
                return true;
            }
            out += body[i];
        }

        return false;
    }
}

config::ConfigServer::ConfigServer() {
    m_registry = nullptr;
    m_modes = nullptr;
    m_server = nullptr;
    m_running = false;
    m_boundPort = 0;
}

config::ConfigServer::~ConfigServer() {
    stop();
}

void config::ConfigServer::initialize(
    const Parameters &params,
    ParameterRegistry *registry,
    DriveModeSet *modes)
{
    m_params = params;
    m_registry = registry;
    m_modes = modes;

    buildSchema();
    publish(TelemetrySample());
}

void config::ConfigServer::buildSchema() {
    std::ostringstream out;
    out << "{\"registry\":";

    if (m_registry != nullptr) m_registry->serializeJson(out);
    else out << "{\"parameters\":[]}";

    out << ",\"modes\":[";
    if (m_modes != nullptr) {
        for (int i = 0; i < m_modes->getCount(); ++i) {
            if (i != 0) out << ',';
            out << jsonString(m_modes->get(i).getName());
        }
    }
    out << "]}";

    std::lock_guard<std::mutex> lock(m_mutex);
    m_schema = out.str();
}

void config::ConfigServer::refreshState(const TelemetrySample &sample) {
    std::ostringstream out;
    out << "{\"values\":{";

    if (m_registry != nullptr) {
        bool first = true;
        for (int i = 0; i < m_registry->getCount(); ++i) {
            const ParameterDescriptor &d = m_registry->getDescriptor(i);
            if (d.type == ParameterType::Map) continue;

            if (!first) out << ',';
            first = false;

            out << jsonString(d.path) << ':' << m_registry->getValue(i);
        }
    }

    out << "},\"telemetry\":{"
        << "\"time\":" << sample.time
        << ",\"engineRpm\":" << sample.engineRpm
        << ",\"throttlePlate\":" << sample.throttlePlate
        << ",\"indicatedTorque\":" << sample.indicatedTorque
        << ",\"torqueRequest\":" << sample.torqueRequest
        << ",\"coolantTemperature\":" << sample.coolantTemperature
        << ",\"oilTemperature\":" << sample.oilTemperature
        << ",\"vehicleSpeed\":" << sample.vehicleSpeed
        << ",\"roadGrade\":" << sample.roadGrade
        << ",\"gear\":" << sample.gear
        << ",\"clutchPressure\":" << sample.clutchPressure
        << ",\"ignitionCut\":" << sample.ignitionCut
        << ",\"fuelCut\":" << sample.fuelCut
        << ",\"idleIntegrator\":" << sample.idleIntegrator
        << ",\"torqueCorrection\":" << sample.torqueCorrection
        << ",\"fuelTrim\":" << sample.fuelTrim
        << ",\"shiftIterations\":" << sample.shiftIterations
        << ",\"shiftErrorNorm\":" << sample.shiftErrorNorm
        << ",\"adaptionEnabled\":" << (sample.adaptionEnabled ? "true" : "false")
        << ",\"selectedMode\":" << sample.selectedMode
        << ",\"engineState\":" << jsonString(sample.engineState)
        << ",\"shiftState\":" << jsonString(sample.shiftState)
        << "}}";

    std::ostringstream exported;
    std::ostringstream overridden;
    if (m_registry != nullptr) {
        m_registry->exportScript(exported, ParameterRegistry::ExportScope::Learned);
        m_registry->exportScript(overridden, ParameterRegistry::ExportScope::Changed);
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_state = out.str();
    m_export = exported.str();
    m_overrides = overridden.str();
}

void config::ConfigServer::publish(const TelemetrySample &sample) {
    refreshState(sample);
}

std::string config::ConfigServer::schemaJson() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_schema;
}

std::string config::ConfigServer::stateJson() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state;
}

std::string config::ConfigServer::exportScript() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_export;
}

std::string config::ConfigServer::exportOverrides() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_overrides;
}

bool config::ConfigServer::queueCommand(const ParameterCommand &command) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_commands.push_back(command);

    return true;
}

int config::ConfigServer::applyPendingCommands() {
    std::vector<ParameterCommand> pending;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        pending.swap(m_commands);
    }

    if (m_registry == nullptr) return 0;

    int applied = 0;
    for (const ParameterCommand &command : pending) {
        switch (command.kind) {
        case ParameterCommand::Kind::SetParameter:
            if (m_registry->set(command.path, command.value)) ++applied;
            break;

        case ParameterCommand::Kind::SelectMode:
            if (m_modes != nullptr
                && m_modes->select(static_cast<int>(command.value), m_registry))
            {
                ++applied;
            }
            break;

        case ParameterCommand::Kind::ResetDefaults:
            m_registry->resetToDefaults();
            ++applied;
            break;
        }
    }

    return applied;
}

bool config::ConfigServer::start() {
    if (m_running) return true;

    httplib::Server *server = new httplib::Server;
    m_server = server;

    server->Get("/api/schema", [this](const httplib::Request &, httplib::Response &res) {
        res.set_content(schemaJson(), "application/json");
    });

    server->Get("/api/state", [this](const httplib::Request &, httplib::Response &res) {
        res.set_content(stateJson(), "application/json");
    });

    server->Get("/api/export", [this](const httplib::Request &, httplib::Response &res) {
        res.set_content(exportScript(), "text/plain");
    });

    server->Get("/api/overrides", [this](const httplib::Request &, httplib::Response &res) {
        res.set_content(exportOverrides(), "text/plain");
    });

    server->Post("/api/set", [this](const httplib::Request &req, httplib::Response &res) {
        ParameterCommand command;
        command.kind = ParameterCommand::Kind::SetParameter;

        if (!extractString(req.body, "path", &command.path)
            || !extractNumber(req.body, "value", &command.value))
        {
            res.status = 400;
            res.set_content("{\"ok\":false}", "application/json");
            return;
        }

        queueCommand(command);
        res.set_content("{\"ok\":true}", "application/json");
    });

    server->Post("/api/mode", [this](const httplib::Request &req, httplib::Response &res) {
        ParameterCommand command;
        command.kind = ParameterCommand::Kind::SelectMode;

        if (!extractNumber(req.body, "index", &command.value)) {
            res.status = 400;
            res.set_content("{\"ok\":false}", "application/json");
            return;
        }

        queueCommand(command);
        res.set_content("{\"ok\":true}", "application/json");
    });

    server->Post("/api/reset", [this](const httplib::Request &, httplib::Response &res) {
        ParameterCommand command;
        command.kind = ParameterCommand::Kind::ResetDefaults;

        queueCommand(command);
        res.set_content("{\"ok\":true}", "application/json");
    });

    server->Get("/", [this](const httplib::Request &, httplib::Response &res) {
        std::ifstream file(m_params.uiPath, std::ios::in | std::ios::binary);
        if (!file.is_open()) {
            res.status = 404;
            res.set_content("configuration ui not found", "text/plain");
            return;
        }

        std::ostringstream contents;
        contents << file.rdbuf();
        res.set_content(contents.str(), "text/html");
    });

    int port = 0;
    if (m_params.port > 0 && server->bind_to_port(m_params.host.c_str(), m_params.port)) {
        port = m_params.port;
    }

    if (port <= 0) port = server->bind_to_any_port(m_params.host.c_str());

    if (port <= 0) {
        delete server;
        m_server = nullptr;
        return false;
    }

    m_boundPort = port;
    m_running = true;

    m_thread = std::thread([server]() {
        server->listen_after_bind();
    });

    server->wait_until_ready();

    return true;
}

void config::ConfigServer::stop() {
    if (!m_running) return;

    httplib::Server *server = serverOf(m_server);
    if (server != nullptr) server->stop();

    if (m_thread.joinable()) m_thread.join();

    delete server;
    m_server = nullptr;
    m_running = false;
    m_boundPort = 0;
}

bool config::ConfigServer::isRunning() const {
    return m_running;
}

int config::ConfigServer::getBoundPort() const {
    return m_boundPort;
}
