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

// End-to-end against a live OPC-UA server (no HW): boot the test_alarm_server
// fixture and exercise the paths that only a real session can reach.
//
// INV2 identity: prove the asset-identity nameplate is filled from the server's
// device-info (ServerStatus/BuildInfo + the OPC-UA DI DeviceSet nameplate) with
// no manual entry, through both the raw OpcuaClient::read_device_info read and
// the full OpcuaPlugin::introspect() path that lands identity on the SOVD
// Component.
//
// Connection lifecycle: prove a successful connect clears the standing
// PLC_COMMS_LOST fault, which needs a connect that actually succeeds.

#include "ros2_medkit_opcua/device_identity.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"
#include "ros2_medkit_opcua/opcua_plugin.hpp"
#include "ros2_medkit_opcua/opcua_poller.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <ros2_medkit_msgs/srv/clear_fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

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
    int stdin_pipe[2];
    if (pipe(stdin_pipe) != 0) {
      close(pipefd[0]);
      close(pipefd[1]);
      return false;
    }
    pid_ = fork();
    if (pid_ < 0) {
      close(pipefd[0]);
      close(pipefd[1]);
      close(stdin_pipe[0]);
      close(stdin_pipe[1]);
      return false;
    }
    if (pid_ == 0) {
      dup2(pipefd[1], STDOUT_FILENO);
      dup2(pipefd[1], STDERR_FILENO);
      dup2(stdin_pipe[0], STDIN_FILENO);
      close(pipefd[0]);
      close(pipefd[1]);
      close(stdin_pipe[0]);
      close(stdin_pipe[1]);
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
    close(stdin_pipe[0]);
    read_fd_ = pipefd[0];
    write_fd_ = stdin_pipe[1];
    return wait_for_ready(15000);
  }

  // One CLI command ("fire Overpressure 750", "clear Overpressure", ...). The
  // fixture reads them line by line off stdin.
  bool send(const std::string & command) {
    if (write_fd_ < 0) {
      return false;
    }
    const std::string line = command + "\n";
    return write(write_fd_, line.c_str(), line.size()) == static_cast<ssize_t>(line.size());
  }

  void stop() {
    if (write_fd_ >= 0) {
      close(write_fd_);
      write_fd_ = -1;
    }
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
  int write_fd_{-1};
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

// A gateway that restarts after a comms outage never raised PLC_COMMS_LOST in
// THIS process, yet the fault manager keys faults by fault_code alone and
// persists them, so the fault raised before the restart is still standing.
// The reconnect arm used to clear only when its own in-memory
// ``comms_lost_raised_`` flag was set, which no restart can satisfy, so the
// fault stayed CONFIRMED for good. The clear now goes out on every successful
// connect. Driven against the live fixture because the arm can only be reached
// by a connect that actually succeeds.
TEST_F(OpcuaIdentityE2ETest, SuccessfulConnectClearsCommsLostNeverRaisedHere) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = endpoint_;
  config.connect_timeout = std::chrono::milliseconds(5000);
  // Connect once to seed the client's stored config (what the poller reconnects
  // with), then drop the session so the poll loop starts in its reconnect arm -
  // the state a freshly started gateway is in while the PLC is already up.
  ASSERT_TRUE(client.connect(config));
  client.disconnect();
  ASSERT_FALSE(client.is_connected());

  NodeMap node_map;  // config-less: no entries, nothing to poll
  OpcuaPoller poller(client, node_map);

  std::mutex signals_mutex;
  std::vector<std::pair<std::string, bool>> signals;  // (fault_code, active)
  poller.set_alarm_callback(
      [&signals_mutex, &signals](const std::string &, const ros2_medkit::fault_detection::FaultSignal & signal) {
        std::lock_guard<std::mutex> lock(signals_mutex);
        signals.emplace_back(signal.fault_code, signal.active);
      });

  PollerConfig poller_config;
  poller_config.poll_interval = std::chrono::milliseconds(100);
  poller_config.reconnect_interval = std::chrono::milliseconds(100);
  poller_config.comms_lost_fault_enabled = true;
  poller.start(poller_config);

  bool cleared = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!cleared && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(signals_mutex);
      cleared = std::find(signals.begin(), signals.end(), std::make_pair(std::string(kCommsLostFaultCode), false)) !=
                signals.end();
    }
    if (!cleared) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  poller.stop();

  EXPECT_TRUE(cleared) << "a successful connect must clear PLC_COMMS_LOST even when this process never raised it";

  // Absence control on the same harness: the connect succeeded, so nothing may
  // have RAISED the fault. Without this a clear-everything-always regression
  // would still pass the assertion above.
  std::lock_guard<std::mutex> lock(signals_mutex);
  EXPECT_EQ(std::find(signals.begin(), signals.end(), std::make_pair(std::string(kCommsLostFaultCode), true)),
            signals.end())
      << "comms-lost must not be raised while the connection is up";
}

namespace {

// RAII rclcpp init/shutdown, tearing down only what it started.
struct ScopedRclcpp {
  const bool owned_;
  ScopedRclcpp() : owned_(!rclcpp::ok()) {
    if (owned_) {
      rclcpp::init(0, nullptr);
    }
  }
  ~ScopedRclcpp() {
    if (owned_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
  ScopedRclcpp(const ScopedRclcpp &) = delete;
  ScopedRclcpp & operator=(const ScopedRclcpp &) = delete;
};

// The plugin only builds its fault-service clients when the context hands it a
// real node, which is what makes the ClearFault request observable on the wire.
class RealNodePluginContext : public FakePluginContext {
 public:
  explicit RealNodePluginContext(rclcpp::Node * node) : node_(node) {
  }
  rclcpp::Node * node() const override {
    return node_;
  }

 private:
  rclcpp::Node * node_;
};

}  // namespace

// The connect-time clear, read off the wire. clear_comms_lost_on_connect() is
// only reachable through a connect that SUCCEEDS, so it needs the live fixture,
// and the flag it sets is only observable with a real fault-manager service on
// the other end. A correlation rule may name PLC_COMMS_LOST as the root cause of
// every symptom an outage produced, and the link coming back is not an operator
// resolving those, so this clear must not cascade.
TEST_F(OpcuaIdentityE2ETest, ConnectTimeCommsLostClearSkipsTheCorrelationCascade) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_connect_clear");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_connect_clear_faultmgr");

  std::mutex received_mutex;
  std::vector<ros2_medkit_msgs::srv::ClearFault::Request> cleared_requests;
  auto report_srv = fault_manager->create_service<ros2_medkit_msgs::srv::ReportFault>(
      "/fault_manager/report_fault", [](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request>,
                                        std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> res) {
        res->accepted = true;
      });
  auto clear_srv = fault_manager->create_service<ros2_medkit_msgs::srv::ClearFault>(
      "/fault_manager/clear_fault",
      [&cleared_requests, &received_mutex](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> req,
                                           std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> res) {
        {
          std::lock_guard<std::mutex> lock(received_mutex);
          cleared_requests.push_back(*req);
        }
        res->success = true;
      });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  const std::string yaml_path = write_minimal_node_map();
  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["node_map_path"] = yaml_path;
  config["poll_interval_ms"] = 100;
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  ctx.entities["tank"] = {SovdEntityType::APP, "tank", "/test_plc", "/test_plc/test_runtime/tank"};
  // The connect inside set_context() succeeds against the fixture, which is the
  // only way to reach the connect-time clear.
  plugin.set_context(ctx);

  // The clear may be buffered until the stub service is DDS-matched. The poll
  // thread drains the buffer on its next cycle.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  bool delivered = false;
  while (!delivered && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(received_mutex);
      delivered = !cleared_requests.empty();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  plugin.shutdown();
  std::remove(yaml_path.c_str());

  std::lock_guard<std::mutex> lock(received_mutex);
  ASSERT_FALSE(cleared_requests.empty()) << "a successful connect sent no ClearFault at all";
  EXPECT_EQ(cleared_requests.front().fault_code, std::string(kCommsLostFaultCode));
  EXPECT_TRUE(cleared_requests.front().skip_correlation_auto_clear)
      << "the connect-time clear cascade-cleared the symptoms of the outage it ended";
}

// The other side of the same rule, also on the wire: when the DEVICE reports its
// condition inactive, that IS a resolution at the source, so the correlation
// engine may act on it and the flag stays off. Only a live AlarmCondition
// lifecycle reaches on_event_alarm's ClearFault arm, so this drives the
// fixture's own CLI to fire and then clear a condition.
TEST_F(OpcuaIdentityE2ETest, DeviceReportedAlarmClearKeepsTheCorrelationCascade) {
  ScopedRclcpp rclcpp_scope;
  auto node = std::make_shared<rclcpp::Node>("opcua_identity_device_clear");
  auto fault_manager = std::make_shared<rclcpp::Node>("opcua_identity_device_clear_faultmgr");

  std::mutex received_mutex;
  std::vector<std::string> reported;
  std::vector<ros2_medkit_msgs::srv::ClearFault::Request> cleared_requests;
  auto report_srv = fault_manager->create_service<ros2_medkit_msgs::srv::ReportFault>(
      "/fault_manager/report_fault",
      [&reported, &received_mutex](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> req,
                                   std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> res) {
        {
          std::lock_guard<std::mutex> lock(received_mutex);
          reported.push_back(req->fault_code);
        }
        res->accepted = true;
      });
  auto clear_srv = fault_manager->create_service<ros2_medkit_msgs::srv::ClearFault>(
      "/fault_manager/clear_fault",
      [&cleared_requests, &received_mutex](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> req,
                                           std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> res) {
        {
          std::lock_guard<std::mutex> lock(received_mutex);
          cleared_requests.push_back(*req);
        }
        res->success = true;
      });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(fault_manager);
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  OpcuaPlugin plugin;
  nlohmann::json config;
  config["endpoint_url"] = endpoint_;
  config["poll_interval_ms"] = 100;
  // Zero-config native A&C on the Server EventNotifier, with auto_clear so the
  // condition going inactive clears the fault without an operator ack/confirm.
  config["auto_alarms"] = nlohmann::json{{"enabled", true}, {"auto_clear", true}};
  plugin.configure(config);

  RealNodePluginContext ctx(node.get());
  plugin.set_context(ctx);

  const auto reported_count = [&received_mutex, &reported]() {
    std::lock_guard<std::mutex> lock(received_mutex);
    return reported.size();
  };
  // The connect-time PLC_COMMS_LOST clear also lands here (this connect
  // succeeded), so a clear is looked up by the code it names.
  const auto clear_for = [&received_mutex, &cleared_requests](const std::string & code) -> std::optional<bool> {
    std::lock_guard<std::mutex> lock(received_mutex);
    for (const auto & req : cleared_requests) {
      if (req.fault_code == code) {
        return req.skip_correlation_auto_clear;
      }
    }
    return std::nullopt;
  };

  // Fire until the event subscription is up and a report lands. The retry is the
  // subscription handshake, not flakiness in the assertion: an event fired
  // before the subscribe simply is not delivered.
  const auto fire_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (reported_count() == 0 && std::chrono::steady_clock::now() < fire_deadline) {
    ASSERT_TRUE(server_.send("fire Overpressure 750"));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  ASSERT_GT(reported_count(), 0u) << "the fixture's AlarmCondition never reached the fault manager";

  std::string alarm_code;
  {
    std::lock_guard<std::mutex> lock(received_mutex);
    alarm_code = reported.front();
  }
  ASSERT_TRUE(server_.send("clear Overpressure"));
  const auto clear_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (!clear_for(alarm_code).has_value() && std::chrono::steady_clock::now() < clear_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  plugin.shutdown();

  const auto device_clear_skips = clear_for(alarm_code);
  ASSERT_TRUE(device_clear_skips.has_value())
      << "the device reporting condition " << alarm_code << " inactive sent no ClearFault";
  EXPECT_FALSE(*device_clear_skips) << "a clear the device itself reported must keep the correlation cascade";

  // The connect-time clear travelled the same wire in the same test, and it is
  // the opposite case: not an operator resolving anything, so it does not
  // cascade. Having both here is what makes the flag above a decision rather
  // than a constant.
  const auto link_state_clear_skips = clear_for(kCommsLostFaultCode);
  ASSERT_TRUE(link_state_clear_skips.has_value()) << "the connect-time clear never arrived";
  EXPECT_TRUE(*link_state_clear_skips);
}

}  // namespace ros2_medkit_gateway
