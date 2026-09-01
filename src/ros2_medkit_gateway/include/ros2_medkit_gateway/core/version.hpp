// Copyright 2026 bburda
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

#pragma once

namespace ros2_medkit_gateway {

/// Gateway version - auto-populated from package.xml via CMake at configure time.
/// Fallback to hardcoded value if GATEWAY_VERSION is not defined (e.g. non-CMake builds).
#ifdef GATEWAY_VERSION_STRING
constexpr const char * kGatewayVersion = GATEWAY_VERSION_STRING;
#else
constexpr const char * kGatewayVersion = "0.7.0";
#endif

/// SOVD specification version
constexpr const char * kSovdVersion = "1.0.0";

}  // namespace ros2_medkit_gateway
