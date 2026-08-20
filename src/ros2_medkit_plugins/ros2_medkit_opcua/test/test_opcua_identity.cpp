// Copyright 2026 mfaferek93
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// INV2 end-to-end (no HW): boot the test_alarm_server OPC-UA fixture, connect,
// and prove the asset-identity nameplate is filled from the server's device-info
// (ServerStatus/BuildInfo + the OPC-UA DI DeviceSet nameplate) with no manual
// entry. Exercises both the raw OpcuaClient::read_device_info read and the full
// OpcuaPlugin::introspect() path that lands identity on the SOVD Component.

#include "ros2_medkit_opcua/device_identity.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"
#include "ros2_medkit_opcua/opcua_plugin.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

#ifndef MEDKIT_ALARM_SERVER_BIN
#define MEDKIT_ALARM_SERVER_BIN ""
#endif

// -- Stub PluginRequest/PluginResponse (mirrors test_opcua_plugin.cpp; the
//    plugin translation unit references them but the HTTP layer is not linked) --

namespace ros2_medkit_gateway {

PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
std::string PluginRequest::path_param(size_t) const {
  return {};
}
std::string PluginRequest::header(const std::string &) const {
  return {};
}
const std::string & PluginRequest::path() const {
  static const std::string empty;
  return empty;
}
const std::string & PluginRequest::body() const {
  static const std::string empty;
  return empty;
}
std::string PluginRequest::query_param(const std::string &) const {
  return {};
}

PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json &) {
}
void PluginResponse::send_error(int, const std::string &, const std::string &, const nlohmann::json &) {
}

// -- FakePluginContext: node() is null (no ROS graph), just enough for
//    set_context() + introspect() to run. --

class FakePluginContext : public RosPluginContext {
 public:
  std::unordered_map<std::string, PluginEntityInfo> entities;

  rclcpp::Node * node() const override {
    return nullptr;
  }
  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    auto it = entities.find(id);
    return it != entities.end() ? std::optional<PluginEntityInfo>(it->second) : std::nullopt;
  }
  std::vector<PluginEntityInfo> get_child_apps(const std::string &) const override {
    return {};
  }
  nlohmann::json list_entity_faults(const std::string &) const override {
    // Contract: a bare JSON array of fault objects (empty for this fake).
    return nlohmann::json::array();
  }
  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest &, PluginResponse &,
                                                            const std::string & entity_id) const override {
    return get_entity(entity_id);
  }
  void register_capability(SovdEntityType, const std::string &) override {
  }
  void register_entity_capability(const std::string &, const std::string &) override {
  }
  std::vector<std::string> get_type_capabilities(SovdEntityType) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string &) const override {
    return {};
  }
  LockAccessResult check_lock(const std::string &, const std::string &, const std::string &) const override {
    return {true, "", "", ""};
  }
  tl::expected<LockInfo, LockError> acquire_lock(const std::string &, const std::string &,
                                                 const std::vector<std::string> &, int) override {
    return tl::make_unexpected(LockError{"not supported", "", 409, std::nullopt});
  }
  tl::expected<void, LockError> release_lock(const std::string &, const std::string &) override {
    return tl::make_unexpected(LockError{"not supported", "", 409, std::nullopt});
  }
  IntrospectionInput get_entity_snapshot() const override {
    return {};
  }
  nlohmann::json list_all_faults() const override {
    return nlohmann::json::object();
  }
  void register_sampler(
      const std::string &,
      const std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> &)
      override {
  }
  ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }
};

namespace {

// Reserve an ephemeral loopback port and release it (best-effort; a race with
// the fixture bind is unlikely on a test host and retried by the caller).
int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  int port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
    socklen_t len = sizeof(addr);
    if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &len) == 0) {
      port = ntohs(addr.sin_port);
    }
  }
  close(sock);
  return port;
}

// Boots the test_alarm_server binary as a child process and blocks until it
// prints the "READY " handshake line on stdout. SIGTERM on teardown.
class AlarmServer {
 public:
  ~AlarmServer() {
    stop();
  }

  bool start(const std::string & binary, int port, const std::vector<std::string> & extra_args = {}) {
    int pipefd[2];
    if (pipe(pipefd) != 0) {
      return false;
    }
    pid_ = fork();
    if (pid_ < 0) {
      close(pipefd[0]);
      close(pipefd[1]);
      return false;
    }
    if (pid_ == 0) {
      dup2(pipefd[1], STDOUT_FILENO);
      dup2(pipefd[1], STDERR_FILENO);
      close(pipefd[0]);
      close(pipefd[1]);
      std::string port_str = std::to_string(port);
      std::vector<const char *> argv_vec{binary.c_str(), "--port", port_str.c_str()};
      for (const auto & arg : extra_args) {
        argv_vec.push_back(arg.c_str());
      }
      argv_vec.push_back(nullptr);
      execv(binary.c_str(), const_cast<char * const *>(argv_vec.data()));
      _exit(127);
    }
    close(pipefd[1]);
    read_fd_ = pipefd[0];
    return wait_for_ready(15000);
  }

  void stop() {
    if (pid_ > 0) {
      kill(pid_, SIGTERM);
      int status = 0;
      waitpid(pid_, &status, 0);
      pid_ = -1;
    }
    if (read_fd_ >= 0) {
      close(read_fd_);
      read_fd_ = -1;
    }
  }

 private:
  bool wait_for_ready(int timeout_ms) {
    std::string acc;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      pollfd pfd{read_fd_, POLLIN, 0};
      int remaining = static_cast<int>(
          std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count());
      int rc = poll(&pfd, 1, remaining > 0 ? remaining : 0);
      if (rc <= 0) {
        continue;
      }
      char buf[256];
      ssize_t n = read(read_fd_, buf, sizeof(buf));
      if (n <= 0) {
        return false;  // EOF: child died before READY
      }
      acc.append(buf, static_cast<size_t>(n));
      if (acc.find("READY ") != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  pid_t pid_{-1};
  int read_fd_{-1};
};

std::string fixture_binary() {
  return std::string(MEDKIT_ALARM_SERVER_BIN);
}

bool fixture_available() {
  const std::string bin = fixture_binary();
  return !bin.empty() && access(bin.c_str(), X_OK) == 0;
}

}  // namespace

// -- Fixture that boots the alarm server once per test --

class OpcuaIdentityE2ETest : public ::testing::Test {
 protected:
  void SetUp() override {
    // The fixture is built by this package's own CMake and is a declared
    // dependency of this target; a missing binary means the build is broken,
    // so fail hard instead of skipping (run_ctest.py does the same).
    ASSERT_TRUE(fixture_available()) << "test_alarm_server fixture missing or not executable at '" << fixture_binary()
                                     << "'";
    port_ = reserve_local_port();
    ASSERT_NE(port_, 0);
    ASSERT_TRUE(server_.start(fixture_binary(), port_)) << "test_alarm_server did not signal READY";
    endpoint_ = "opc.tcp://127.0.0.1:" + std::to_string(port_);
    // The fixture prints READY before the OPC-UA listen socket is fully
    // accepting, so probe until a real connection succeeds. Once connectable it
    // stays so, making every per-test connect (and the plugin's) race-free.
    ASSERT_TRUE(wait_until_connectable()) << "fixture never became connectable at " << endpoint_;
  }

  void TearDown() override {
    server_.stop();
  }

  bool wait_until_connectable() {
    for (int attempt = 0; attempt < 50; ++attempt) {
      OpcuaClient probe;
      OpcuaClientConfig config;
      config.endpoint_url = endpoint_;
      config.connect_timeout = std::chrono::milliseconds(1000);
      if (probe.connect(config)) {
        probe.disconnect();
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  // Tear the fixture down and boot a fresh instance on the SAME port with
  // different CLI args (e.g. a new --serial). Simulates a PLC reboot /
  // device swap for the per-session identity refresh test.
  bool restart_server(const std::vector<std::string> & extra_args) {
    server_.stop();
    if (!server_.start(fixture_binary(), port_, extra_args)) {
      return false;
    }
    return wait_until_connectable();
  }

  AlarmServer server_;
  int port_{0};
  std::string endpoint_;
};

TEST_F(OpcuaIdentityE2ETest, ClientReadsServerBuildInfo) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  ASSERT_TRUE(client.connect(config));

  auto info = client.read_device_info();
  // The fixture pins explicit BuildInfo values.
  EXPECT_EQ(info.manufacturer_name, "SelfPatch Test Manufacturer");
  EXPECT_EQ(info.product_name, "SelfPatch Test PLC");
  EXPECT_EQ(info.software_version, "1.2.3");
  EXPECT_EQ(info.build_number, "build-4567");
  client.disconnect();
}

TEST_F(OpcuaIdentityE2ETest, ClientReadsDiNameplate) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  ASSERT_TRUE(client.connect(config));

  auto info = client.read_device_info();
  // The fixture exposes an OPC-UA DI DeviceSet nameplate.
  EXPECT_EQ(info.di_manufacturer, "SelfPatch Devices");
  EXPECT_EQ(info.di_model, "SPX-1000");
  EXPECT_EQ(info.di_serial_number, "SN-0001-TEST");
  EXPECT_EQ(info.di_hardware_revision, "HW-A2");
  EXPECT_EQ(info.di_software_revision, "SW-3.4.5");
  // OrderNumber lives in the vendor namespace (not DI) with leading + trailing
  // pad; it is matched by BrowseName across namespaces and edge-trimmed while the
  // load-bearing internal space in the MLFB is preserved.
  EXPECT_EQ(info.di_order_number, "6ES7 672-5SC11-0YA0");
  client.disconnect();
}

TEST_F(OpcuaIdentityE2ETest, MappedIdentityFromLiveServer) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  ASSERT_TRUE(client.connect(config));

  auto id = opcua_device_info_to_identity(client.read_device_info(), endpoint_);
  // DI nameplate wins over BuildInfo for manufacturer / model / software.
  EXPECT_EQ(id.manufacturer, "SelfPatch Devices");
  EXPECT_EQ(id.model, "SPX-1000");
  EXPECT_EQ(id.order_code, "6ES7 672-5SC11-0YA0");
  EXPECT_EQ(id.serial_number, "SN-0001-TEST");
  EXPECT_EQ(id.hardware_revision, "HW-A2");
  EXPECT_EQ(id.software_version, "SW-3.4.5");
  EXPECT_EQ(id.network_endpoint, endpoint_);
  EXPECT_EQ(id.extra.at("buildNumber"), "build-4567");
  EXPECT_EQ(id.provenance.at("serial_number"), "opcua");
  EXPECT_EQ(id.provenance.at("order_code"), "opcua");
  EXPECT_EQ(id.provenance.at("network_endpoint"), "opcua");
  client.disconnect();
}

namespace {

// Minimal node map so introspect() can name the area / component. The single
// node points at a nonexistent address-space node; the poller's failed reads
// do not drop the connection (BadNodeIdUnknown != disconnect).
std::string write_minimal_node_map() {
  const std::string yaml_path = "/tmp/test_opcua_identity_nodemap.yaml";
  std::ofstream f(yaml_path);
  f << R"(
area_id: test_plc
area_name: Test PLC Area
component_id: test_runtime
component_name: Test PLC Runtime
nodes:
  - node_id: "ns=2;i=9999"
    entity_id: tank
    data_name: level
    display_name: Tank Level
    data_type: float
    writable: false
)";
  return yaml_path;
}

}  // namespace

TEST_F(OpcuaIdentityE2ETest, PluginIntrospectPopulatesIdentity) {
  const std::string yaml_path = write_minimal_node_map();

  ros2_medkit_gateway::OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = yaml_path;
  config["endpoint_url"] = endpoint_;
  plugin.configure(config);

  FakePluginContext ctx;
  plugin.set_context(ctx);

  auto result = plugin.introspect(IntrospectionInput{});
  ASSERT_FALSE(result.new_entities.components.empty());

  const auto & comp = result.new_entities.components.front();
  // The fixture session is unsecured (SecurityPolicy=None), so the component
  // gets the generic "plugin" tag: the spoofable nameplate may fill gaps in an
  // operator manifest but never override it. Per-field provenance still says
  // "opcua" so the read origin stays visible.
  EXPECT_EQ(comp.source, "plugin");
  ASSERT_FALSE(comp.identity.empty()) << "identity should be filled from the OPC-UA device-info";
  EXPECT_EQ(comp.identity.manufacturer, "SelfPatch Devices");
  EXPECT_EQ(comp.identity.model, "SPX-1000");
  EXPECT_EQ(comp.identity.serial_number, "SN-0001-TEST");
  EXPECT_EQ(comp.identity.network_endpoint, endpoint_);
  EXPECT_EQ(comp.identity.provenance.at("manufacturer"), "opcua");

  // Serialized SOVD JSON carries the nameplate under x-medkit.identity.
  auto j = comp.to_json();
  ASSERT_TRUE(j["x-medkit"].contains("identity"));
  EXPECT_EQ(j["x-medkit"]["identity"]["serialNumber"], "SN-0001-TEST");
  EXPECT_EQ(j["x-medkit"]["identity"]["_provenance"]["manufacturer"], "opcua");

  std::remove(yaml_path.c_str());
}

TEST_F(OpcuaIdentityE2ETest, PluginSourceTagIsTrustGated) {
  const std::string yaml_path = write_minimal_node_map();

  // Secured + certificate-validated profile: the protocol tag "opcua" is
  // stamped, giving the nameplate authority over the manifest. The cert paths
  // do not exist, so the connect fails fast without contacting a server - the
  // source tag is decided by configuration, not by connection state.
  ros2_medkit_gateway::OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = yaml_path;
  config["endpoint_url"] = endpoint_;
  config["security_policy"] = "Basic256Sha256";
  config["security_mode"] = "SignAndEncrypt";
  config["client_cert_path"] = "/nonexistent/client_cert.der";
  config["client_key_path"] = "/nonexistent/client_key.pem";
  config["reject_untrusted"] = true;
  plugin.configure(config);

  FakePluginContext ctx;
  plugin.set_context(ctx);

  auto result = plugin.introspect(IntrospectionInput{});
  ASSERT_FALSE(result.new_entities.components.empty());
  EXPECT_EQ(result.new_entities.components.front().source, "opcua");

  // Same secured profile but accept-any server cert: a rogue endpoint would be
  // accepted, so the identity authority drops back to the generic "plugin" tag.
  ros2_medkit_gateway::OpcuaPlugin accept_any_plugin;
  config["reject_untrusted"] = false;
  accept_any_plugin.configure(config);
  FakePluginContext ctx2;
  accept_any_plugin.set_context(ctx2);
  auto accept_any_result = accept_any_plugin.introspect(IntrospectionInput{});
  ASSERT_FALSE(accept_any_result.new_entities.components.empty());
  EXPECT_EQ(accept_any_result.new_entities.components.front().source, "plugin");

  std::remove(yaml_path.c_str());
}

TEST_F(OpcuaIdentityE2ETest, IdentityRefreshedAfterReconnect) {
  const std::string yaml_path = write_minimal_node_map();

  ros2_medkit_gateway::OpcuaPlugin plugin;
  nlohmann::json config;
  config["node_map_path"] = yaml_path;
  config["endpoint_url"] = endpoint_;
  plugin.configure(config);

  FakePluginContext ctx;
  plugin.set_context(ctx);

  auto result = plugin.introspect(IntrospectionInput{});
  ASSERT_FALSE(result.new_entities.components.empty());
  ASSERT_EQ(result.new_entities.components.front().identity.serial_number, "SN-0001-TEST");

  // Reboot the "PLC" on the same port with a different nameplate. The plugin's
  // poller detects the drop and reconnects in the background; the next
  // introspect on the new session must re-read the device-info instead of
  // serving the value latched from the first session.
  ASSERT_TRUE(restart_server({"--serial", "SN-0002-RECONNECT"}));

  std::string observed_serial;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (std::chrono::steady_clock::now() < deadline) {
    auto refreshed = plugin.introspect(IntrospectionInput{});
    ASSERT_FALSE(refreshed.new_entities.components.empty());
    observed_serial = refreshed.new_entities.components.front().identity.serial_number;
    if (observed_serial == "SN-0002-RECONNECT") {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  EXPECT_EQ(observed_serial, "SN-0002-RECONNECT") << "identity not refreshed after reconnect";

  std::remove(yaml_path.c_str());
}

TEST_F(OpcuaIdentityE2ETest, ClientConnectionGenerationCountsSessions) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);

  EXPECT_EQ(client.connection_generation(), 0u);
  ASSERT_TRUE(client.connect(config));
  EXPECT_EQ(client.connection_generation(), 1u);
  // Re-connect on an already-open session is not a new session.
  ASSERT_TRUE(client.connect(config));
  EXPECT_EQ(client.connection_generation(), 1u);

  client.disconnect();
  ASSERT_TRUE(client.connect(config));
  EXPECT_EQ(client.connection_generation(), 2u);
  client.disconnect();
}

TEST_F(OpcuaIdentityE2ETest, DiNameplateReadFollowsBrowseContinuationPoints) {
  // Cap the server at 2 references per Browse result: every folder on the DI
  // nameplate path (ObjectsFolder, DeviceSet, TestDevice) now pages through
  // BrowseNext continuation points. Without continuation handling the
  // DeviceSet lookup truncates after the first two ObjectsFolder children and
  // the DI fields come back empty.
  ASSERT_TRUE(restart_server({"--max-refs-per-node", "2"}));

  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  ASSERT_TRUE(client.connect(config));

  auto info = client.read_device_info();
  EXPECT_EQ(info.di_manufacturer, "SelfPatch Devices");
  EXPECT_EQ(info.di_model, "SPX-1000");
  EXPECT_EQ(info.di_serial_number, "SN-0001-TEST");
  EXPECT_EQ(info.di_hardware_revision, "HW-A2");
  EXPECT_EQ(info.di_software_revision, "SW-3.4.5");
  EXPECT_EQ(info.di_order_number, "6ES7 672-5SC11-0YA0");
  client.disconnect();
}

}  // namespace ros2_medkit_gateway
