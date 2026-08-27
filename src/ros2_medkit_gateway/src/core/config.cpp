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

#include "ros2_medkit_gateway/core/config.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace ros2_medkit_gateway {

// Helper function to check if a file exists and is readable
static bool file_exists(const std::string & path) {
  if (path.empty()) {
    return false;
  }
  std::error_code ec;
  return std::filesystem::exists(path, ec) && std::filesystem::is_regular_file(path, ec);
}

// TlsConfig implementation

std::string TlsConfig::validate() const {
  if (!enabled) {
    return "";  // Disabled config is always valid
  }

  // Certificate file is required when TLS is enabled
  if (cert_file.empty()) {
    return "TLS: cert_file is required when TLS is enabled";
  }
  if (!file_exists(cert_file)) {
    return "TLS: cert_file does not exist or is not readable: " + cert_file;
  }

  // Key file is required when TLS is enabled
  if (key_file.empty()) {
    return "TLS: key_file is required when TLS is enabled";
  }
  if (!file_exists(key_file)) {
    return "TLS: key_file does not exist or is not readable: " + key_file;
  }

  // CA file is optional, but if provided must exist
  if (!ca_file.empty() && !file_exists(ca_file)) {
    return "TLS: ca_file does not exist or is not readable: " + ca_file;
  }

  // Validate minimum TLS version
  if (min_version != "1.2" && min_version != "1.3") {
    return "TLS: min_version must be '1.2' or '1.3', got: " + min_version;
  }

  return "";  // Valid
}

// TlsConfigBuilder implementation

TlsConfigBuilder & TlsConfigBuilder::with_enabled(bool enabled) {
  config_.enabled = enabled;
  return *this;
}

TlsConfigBuilder & TlsConfigBuilder::with_cert_file(const std::string & cert_file) {
  config_.cert_file = cert_file;
  return *this;
}

TlsConfigBuilder & TlsConfigBuilder::with_key_file(const std::string & key_file) {
  config_.key_file = key_file;
  return *this;
}

TlsConfigBuilder & TlsConfigBuilder::with_ca_file(const std::string & ca_file) {
  config_.ca_file = ca_file;
  return *this;
}

TlsConfigBuilder & TlsConfigBuilder::with_min_version(const std::string & min_version) {
  config_.min_version = min_version;
  return *this;
}

// TODO(future): Add with_mutual_tls when implemented
// TlsConfigBuilder & TlsConfigBuilder::with_mutual_tls(bool mutual_tls) {
//   config_.mutual_tls = mutual_tls;
//   return *this;
// }

TlsConfig TlsConfigBuilder::build() {
  std::string error = config_.validate();
  if (!error.empty()) {
    throw std::invalid_argument(error);
  }
  return config_;
}

// CorsConfigBuilder implementation

CorsConfigBuilder & CorsConfigBuilder::with_origins(std::vector<std::string> origins) {
  config_.allowed_origins = std::move(origins);
  return *this;
}

CorsConfigBuilder & CorsConfigBuilder::with_methods(std::vector<std::string> methods) {
  config_.allowed_methods = std::move(methods);
  return *this;
}

CorsConfigBuilder & CorsConfigBuilder::with_headers(std::vector<std::string> headers) {
  config_.allowed_headers = std::move(headers);
  return *this;
}

CorsConfigBuilder & CorsConfigBuilder::with_credentials(bool credentials) {
  config_.allow_credentials = credentials;
  return *this;
}

CorsConfigBuilder & CorsConfigBuilder::with_max_age(int seconds) {
  config_.max_age_seconds = seconds;
  return *this;
}

CorsConfig CorsConfigBuilder::build() {
  // Filter out empty strings from allowed_origins (used as placeholder for empty list in YAML)
  config_.allowed_origins.erase(std::remove_if(config_.allowed_origins.begin(), config_.allowed_origins.end(),
                                               [](const std::string & s) {
                                                 return s.empty();
                                               }),
                                config_.allowed_origins.end());

  // Enable CORS only if origins are configured
  config_.enabled = !config_.allowed_origins.empty();

  if (config_.enabled) {
    // Validate: credentials cannot be used with wildcard origin
    if (config_.allow_credentials) {
      auto has_wildcard = std::find(config_.allowed_origins.begin(), config_.allowed_origins.end(), "*");
      if (has_wildcard != config_.allowed_origins.end()) {
        throw std::invalid_argument("CORS: allow_credentials cannot be true when allowed_origins contains '*'");
      }
    }

    // Build cached header strings
    for (const auto & method : config_.allowed_methods) {
      if (!config_.methods_header.empty()) {
        config_.methods_header += ", ";
      }
      config_.methods_header += method;
    }

    for (const auto & header : config_.allowed_headers) {
      if (!config_.headers_header.empty()) {
        config_.headers_header += ", ";
      }
      config_.headers_header += header;
    }
  }

  return config_;
}

}  // namespace ros2_medkit_gateway
