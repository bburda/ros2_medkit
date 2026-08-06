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

#include "ros2_medkit_opcua/device_identity.hpp"

#include <cctype>

namespace ros2_medkit_gateway {

namespace {
constexpr const char * kOpcuaSource = "opcua";

/// Lowercase underscore slug: alnum kept, every other run collapsed to a
/// single '_', leading/trailing '_' trimmed. Yields a URL-safe SOVD id.
std::string slugify(const std::string & in) {
  std::string out;
  out.reserve(in.size());
  bool pending_sep = false;
  for (char ch : in) {
    const unsigned char c = static_cast<unsigned char>(ch);
    if (std::isalnum(c)) {
      if (pending_sep && !out.empty()) {
        out += '_';
      }
      pending_sep = false;
      out += static_cast<char>(std::tolower(c));
    } else {
      pending_sep = true;
    }
  }
  return out;
}

/// Extract the host from an ``opc.tcp://host:port/path`` endpoint URL.
std::string host_from_endpoint(const std::string & endpoint_url) {
  std::string s = endpoint_url;
  const auto scheme = s.find("://");
  if (scheme != std::string::npos) {
    s = s.substr(scheme + 3);
  }
  // Bracketed IPv6 literal (opc.tcp://[fe80::1]:4840): the host is the
  // bracket body - find_first_of(":") would truncate inside the address.
  if (!s.empty() && s.front() == '[') {
    const auto close = s.find(']');
    return close != std::string::npos ? s.substr(1, close - 1) : s.substr(1);
  }
  const auto end = s.find_first_of(":/");
  if (end != std::string::npos) {
    s = s.substr(0, end);
  }
  return s;
}

/// Join two nameplate fields with a single space, skipping empties.
std::string join_nonempty(const std::string & a, const std::string & b) {
  if (a.empty()) {
    return b;
  }
  if (b.empty()) {
    return a;
  }
  return a + " " + b;
}
}  // namespace

AssetIdentity opcua_device_info_to_identity(const OpcuaClient::DeviceInfo & info, const std::string & endpoint_url) {
  AssetIdentity identity;
  auto set_field = [&](std::string AssetIdentity::*member, const std::string & value, const char * prov_key) {
    if (!value.empty()) {
      identity.*member = value;
      identity.provenance[prov_key] = kOpcuaSource;
    }
  };

  // DI nameplate (per-device) wins over the server-level BuildInfo.
  const std::string & manufacturer = !info.di_manufacturer.empty() ? info.di_manufacturer : info.manufacturer_name;
  const std::string & model = !info.di_model.empty() ? info.di_model : info.product_name;
  const std::string & software_version =
      !info.di_software_revision.empty() ? info.di_software_revision : info.software_version;

  set_field(&AssetIdentity::manufacturer, manufacturer, "manufacturer");
  set_field(&AssetIdentity::model, model, "model");
  set_field(&AssetIdentity::order_code, info.di_order_number, "order_code");
  set_field(&AssetIdentity::serial_number, info.di_serial_number, "serial_number");
  set_field(&AssetIdentity::hardware_revision, info.di_hardware_revision, "hardware_revision");
  set_field(&AssetIdentity::software_version, software_version, "software_version");
  set_field(&AssetIdentity::network_endpoint, endpoint_url, "network_endpoint");

  if (!info.build_number.empty()) {
    identity.extra["buildNumber"] = info.build_number;
    identity.provenance["extra.buildNumber"] = kOpcuaSource;
  }

  return identity;
}

ComponentIdentity derive_component_identity(const OpcuaClient::DeviceInfo & info, const std::string & endpoint_url) {
  // 1. DI nameplate (per-device, most specific): Manufacturer + Model.
  std::string name = join_nonempty(info.di_manufacturer, info.di_model);

  // 2. Server BuildInfo (OPC-UA ApplicationName/ProductName equivalent).
  if (name.empty()) {
    name = join_nonempty(info.manufacturer_name, info.product_name);
  }

  if (!name.empty()) {
    std::string id = slugify(name);
    if (!id.empty()) {
      return {id, name};
    }
  }

  // 3. Neutral endpoint-derived fallback. NEVER a fixed product string.
  // Slugify the host so a config-less IPv4 (opcua-192.168.1.10, dotted) yields
  // a valid entity id (opcua-192_168_1_10); the raw dotted form is rejected by
  // is_valid_entity_id and the whole Component would be silently dropped.
  const std::string host = host_from_endpoint(endpoint_url);
  if (!host.empty()) {
    const std::string fallback = "opcua-" + slugify(host);
    return {fallback, fallback};
  }

  return {};
}

bool opcua_identity_trusted(const OpcuaClientConfig & config) {
  return OpcuaClient::requires_secure_channel(config) && config.reject_untrusted;
}

}  // namespace ros2_medkit_gateway
