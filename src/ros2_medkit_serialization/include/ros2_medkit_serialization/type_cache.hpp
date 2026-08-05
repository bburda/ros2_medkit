// Copyright 2026 Selfpatch
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

#ifndef ROS2_MEDKIT_SERIALIZATION__TYPE_CACHE_HPP_
#define ROS2_MEDKIT_SERIALIZATION__TYPE_CACHE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "ros2_medkit_serialization/vendored/dynmsg/typesupport.hpp"

namespace ros2_medkit_serialization {

/// Thread-safe cache for ROS 2 type introspection data.
///
/// This class wraps dynmsg's type loading functions and caches the results
/// to avoid repeated dynamic library lookups. It is implemented as a singleton
/// for global access throughout the gateway.
///
/// @note All public methods are thread-safe.
class TypeCache {
 public:
  /// Get the singleton instance
  static TypeCache & instance();

  /// Delete copy constructor
  TypeCache(const TypeCache &) = delete;

  /// Delete copy assignment
  TypeCache & operator=(const TypeCache &) = delete;

  /// Delete move constructor
  ///
  /// Already suppressed by the copy declarations above; spelled out so the
  /// class declares the whole set rather than leaving two members to be
  /// inferred.
  TypeCache(TypeCache &&) = delete;

  /// Delete move assignment
  TypeCache & operator=(TypeCache &&) = delete;

  /// Defaulted destructor
  ///
  /// The only instance is the function-local static in `instance()`, so this
  /// runs at static destruction exactly as the implicit destructor did, over
  /// the same members in the same order. `cache_` holds non-owning pointers
  /// into dynmsg's type supports, so there is nothing here to release.
  ~TypeCache() = default;

  /// Get type info for a message type (C++ introspection)
  ///
  /// @param package_name Package name (e.g., "std_msgs")
  /// @param type_name Type name without prefix (e.g., "String")
  /// @return Pointer to type info, or nullptr if not found
  const TypeInfo_Cpp * get_message_type_info(const std::string & package_name, const std::string & type_name);

  /// Get type info for any interface type (msg/srv/action)
  ///
  /// @param package_name Package name (e.g., "std_srvs")
  /// @param interface_type Interface type ("msg", "srv", or "action")
  /// @param type_name Type name (e.g., "Trigger_Request")
  /// @return Pointer to type info, or nullptr if not found
  const TypeInfo_Cpp * get_message_type_info(const std::string & package_name, const std::string & interface_type,
                                             const std::string & type_name);

  /// Get type info from full type string
  ///
  /// @param full_type Full type string (e.g., "std_msgs/msg/String" or "std_srvs/srv/Trigger_Request")
  /// @return Pointer to type info, or nullptr if not found
  const TypeInfo_Cpp * get_message_type_info(const std::string & full_type);

  /// Parse a full type string into package, interface type, and type name
  ///
  /// @param full_type Full type string (e.g., "std_msgs/msg/String")
  /// @return Tuple of (package_name, interface_type, type_name), or nullopt if invalid format
  static std::optional<std::tuple<std::string, std::string, std::string>>
  parse_type_string(const std::string & full_type);

  /// Check if a type is cached
  ///
  /// @param package_name Package name
  /// @param type_name Type name
  /// @return true if the type is in the cache
  bool is_cached(const std::string & package_name, const std::string & type_name) const;

  /// Clear the cache
  void clear();

  /// Get the number of cached types
  size_t size() const;

 private:
  TypeCache() = default;

  /// Generate cache key from package and type name (assumes "msg" interface type)
  static std::string make_key(const std::string & package_name, const std::string & type_name);

  /// Generate cache key from package, interface type, and type name
  static std::string make_key(const std::string & package_name, const std::string & interface_type,
                              const std::string & type_name);

  mutable std::shared_mutex mutex_;
  std::unordered_map<std::string, const TypeInfo_Cpp *> cache_;
};

}  // namespace ros2_medkit_serialization

#endif  // ROS2_MEDKIT_SERIALIZATION__TYPE_CACHE_HPP_
