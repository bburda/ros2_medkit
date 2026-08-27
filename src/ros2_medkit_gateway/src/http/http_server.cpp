// Copyright 2025 bburda
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

#include "ros2_medkit_gateway/core/http/http_server.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
// TLS1_2_VERSION / TLS1_3_VERSION and SSL_CTX_set_min_proto_version.
#include <openssl/ssl.h>
#endif

namespace ros2_medkit_gateway {

HttpServerManager::HttpServerManager(const TlsConfig & tls_config, std::size_t thread_pool_size,
                                     std::time_t keep_alive_timeout_sec)
  : tls_config_(tls_config), thread_pool_size_(thread_pool_size), keep_alive_timeout_sec_(keep_alive_timeout_sec) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled) {
    // A non-empty ca_file turns on mutual TLS. The SSLServer constructor does
    // the work itself: given a client CA path it calls
    // SSL_CTX_load_verify_locations and then
    // SSL_CTX_set_verify(SSL_VERIFY_PEER | SSL_VERIFY_FAIL_IF_NO_PEER_CERT).
    //
    // That pairing is why this is all-or-nothing per gateway: with a CA set,
    // a client that presents NO certificate is rejected at the handshake.
    // There is no "verify it if offered" middle setting without patching the
    // vendored header. Leaving ca_file empty keeps ordinary server-only TLS,
    // which is what the SOVD bearer-token flow expects.
    ssl_server_ =
        std::make_unique<httplib::SSLServer>(tls_config_.cert_file.c_str(), tls_config_.key_file.c_str(),
                                             tls_config_.ca_file.empty() ? nullptr : tls_config_.ca_file.c_str());

    if (!ssl_server_->is_valid()) {
      throw std::runtime_error(
          "Failed to create SSL server. Check certificate and key files: " + tls_config_.cert_file +
          " (key configured: " + (tls_config_.key_file.empty() ? "no" : "yes") + ")");
    }

    // Configure additional TLS settings
    configure_tls();
    apply_thread_pool(*ssl_server_);
    apply_keep_alive(*ssl_server_);

    RCLCPP_INFO(rclcpp::get_logger("http_server"),
                "TLS/HTTPS enabled - cert: %s, min_version: %s, client certificates: %s", tls_config_.cert_file.c_str(),
                tls_config_.min_version.c_str(),
                tls_config_.ca_file.empty() ? "not required" : "REQUIRED (mutual TLS)");
    // Note: key_file path intentionally not logged for security reasons
  } else {
    server_ = std::make_unique<httplib::Server>();
    apply_thread_pool(*server_);
    apply_keep_alive(*server_);
    RCLCPP_DEBUG(rclcpp::get_logger("http_server"), "TLS/HTTPS disabled - using plain HTTP");
  }
#else
  if (tls_config_.enabled) {
    throw std::runtime_error(
        "TLS/HTTPS support requested but cpp-httplib was not compiled with OpenSSL support. "
        "Ensure CPPHTTPLIB_OPENSSL_SUPPORT is defined.");
  }
  server_ = std::make_unique<httplib::Server>();
  apply_thread_pool(*server_);
  apply_keep_alive(*server_);
#endif
}

void HttpServerManager::apply_thread_pool(httplib::Server & srv) const {
  if (thread_pool_size_ == 0) {
    return;  // Keep the cpp-httplib default task queue.
  }
  // new_task_queue is invoked once when the server starts accepting connections.
  // Returning a fixed-size ThreadPool bounds the request worker count regardless
  // of host CPU count (issue #440). cpp-httplib owns and deletes the returned
  // TaskQueue.
  const std::size_t n = thread_pool_size_;
  srv.new_task_queue = [n] {
    return new httplib::ThreadPool(n);
  };
}

void HttpServerManager::apply_keep_alive(httplib::Server & srv) const {
  if (keep_alive_timeout_sec_ <= 0) {
    return;  // Keep the cpp-httplib default (5s).
  }
  // Bound how long a request-pool worker stays parked on an idle keep-alive
  // connection. With a small fixed pool, the library default (5s) lets a burst
  // of short-lived client connections pin every worker, stalling ordinary
  // requests for up to one timeout per cycle (issue #440).
  srv.set_keep_alive_timeout(keep_alive_timeout_sec_);
}

httplib::Server * HttpServerManager::get_server() {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_) {
    return ssl_server_.get();
  }
#endif
  return server_.get();
}

void HttpServerManager::listen(const std::string & host, int port) {
  std::string protocol = tls_config_.enabled ? "HTTPS" : "HTTP";
  RCLCPP_INFO(rclcpp::get_logger("http_server"), "Starting %s server on %s:%d...", protocol.c_str(), host.c_str(),
              port);

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_) {
    ssl_server_->listen(host, port);
    return;
  }
#endif
  if (server_) {
    server_->listen(host, port);
  }
}

void HttpServerManager::stop() {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_ && ssl_server_->is_running()) {
    RCLCPP_INFO(rclcpp::get_logger("http_server"), "Stopping HTTPS server...");
    ssl_server_->stop();
    return;
  }
#endif
  if (server_ && server_->is_running()) {
    RCLCPP_INFO(rclcpp::get_logger("http_server"), "Stopping HTTP server...");
    server_->stop();
  }
}

bool HttpServerManager::is_running() const {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  if (tls_config_.enabled && ssl_server_) {
    return ssl_server_->is_running();
  }
#endif
  return server_ && server_->is_running();
}

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
void HttpServerManager::configure_tls() {
  if (!ssl_server_) {
    return;
  }

  // Set the protocol floor on our own context rather than inheriting one.
  //
  // Two reasons it has to be us. The SSLServer constructor calls
  // SSL_CTX_set_min_proto_version(ctx_, TLS1_1_VERSION), so the library asks
  // for a floor of TLS 1.1. What a deployment actually gets on top of that is
  // whatever the local OpenSSL policy allows, which differs between the
  // distributions we ship for. Neither of those is a decision this project
  // made, and SOVD requires TLS 1.2 as the minimum, so the value is set here
  // where it can be read and tested.
  //
  // The string is validated in TlsConfig::validate(), which rejects anything
  // other than "1.2" or "1.3" before a server is ever constructed.
  const int min_proto = (tls_config_.min_version == "1.3") ? TLS1_3_VERSION : TLS1_2_VERSION;
  SSL_CTX * ctx = ssl_server_->ssl_context();
  if (ctx == nullptr || SSL_CTX_set_min_proto_version(ctx, min_proto) != 1) {
    // Refuse to serve rather than fall back to the library floor: a caller
    // that asked for 1.3 and silently got 1.1 is worse off than one that got
    // an error, because nothing downstream can tell the difference.
    throw std::runtime_error("Failed to set the minimum TLS version to " + tls_config_.min_version);
  }

  // Log TLS handshake failures for debugging
  ssl_server_->set_logger([](const httplib::Request & req, const httplib::Response & res) {
    if (res.status >= 400) {
      RCLCPP_DEBUG(rclcpp::get_logger("http_server"), "Request %s %s -> %d", req.method.c_str(), req.path.c_str(),
                   res.status);
    }
  });
}
#else
void HttpServerManager::configure_tls() {
  // No-op when OpenSSL support is not available
}
#endif

}  // namespace ros2_medkit_gateway
