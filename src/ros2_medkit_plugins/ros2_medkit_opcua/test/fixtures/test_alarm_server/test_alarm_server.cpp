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

// Standalone OPC UA test fixture: emits AlarmConditionType events for
// integration testing of the ros2_medkit_opcua native alarm subscription
// code path. OpenPLC does not implement AlarmConditionType (only
// OffNormalAlarmType-derived demos with no Acknowledge / Confirm methods),
// so we build this companion server to cover the full Part 9 surface.
//
// Reads commands from stdin, one per line; writes "OK <name>" or
// "ERR <reason>" to stdout for the test harness to poll. See
// docs in the header comment of fire(), clear(), ack(), etc. below.

#include <open62541/plugin/accesscontrol_default.h>
#include <open62541/plugin/log_stdout.h>
#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <open62541/types_generated.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <string>
#include <thread>

namespace {

constexpr const char * NS_URI = "urn:test:alarms";

struct Condition {
  std::string name;
  UA_NodeId node{};
  UA_NodeId source{};
  // Local mirror of the OPC-UA state we expose. Kept in lockstep with the
  // condition node by every handler so the test harness can grep one line
  // for the truth instead of running a separate OPC-UA browse round-trip.
  bool active{false};
  bool acked{true};
  bool confirmed{true};
  bool enabled{true};
  bool shelved{false};
  bool retain{false};
};

void log_state(const Condition & c) {
  std::cout << "STATE " << c.name << " active=" << (c.active ? "true" : "false")
            << " acked=" << (c.acked ? "true" : "false") << " confirmed=" << (c.confirmed ? "true" : "false")
            << " enabled=" << (c.enabled ? "true" : "false") << " shelved=" << (c.shelved ? "true" : "false")
            << " retain=" << (c.retain ? "true" : "false") << std::endl;
}

std::map<std::string, Condition> g_conditions;
std::mutex g_mutex;
std::atomic<bool> g_running{true};

void stop_handler(int) {
  g_running = false;
}

UA_StatusCode add_source(UA_Server * server, const std::string & name, UA_UInt16 ns, UA_NodeId * out) {
  UA_ObjectAttributes attr = UA_ObjectAttributes_default;
  attr.eventNotifier = 1;
  std::string display = name + "Source";
  attr.displayName = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>(display.c_str()));
  UA_QualifiedName qname = UA_QUALIFIEDNAME(ns, const_cast<char *>(display.c_str()));
  // Use a predictable string NodeId ``Alarms.<name>`` in the user namespace so
  // the gateway's ``alarm_source: "ns=2;s=Alarms.Overpressure"`` config can
  // address this exact node. Auto-assigned numeric IDs (the previous form)
  // would not be reproducible across server restarts and would force the test
  // harness to browse-resolve at runtime.
  std::string source_id = "Alarms." + name;
  UA_NodeId requested = UA_NODEID_STRING_ALLOC(ns, source_id.c_str());
  UA_StatusCode rc = UA_Server_addObjectNode(server, requested, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                             UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES), qname,
                                             UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE), attr, nullptr, out);
  UA_NodeId_clear(&requested);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  // The condition source must be a notifier of the Server object so that the
  // A&C subsystem can route events through the standard notification path.
  UA_ExpandedNodeId target;
  UA_ExpandedNodeId_init(&target);
  UA_NodeId_copy(out, &target.nodeId);
  rc = UA_Server_addReference(server, UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER), UA_NODEID_NUMERIC(0, UA_NS0ID_HASNOTIFIER),
                              target, UA_TRUE);
  UA_ExpandedNodeId_clear(&target);
  return rc;
}

UA_StatusCode add_condition(UA_Server * server, const std::string & name, UA_UInt16 ns, Condition & out) {
  out.name = name;
  UA_StatusCode rc = add_source(server, name, ns, &out.source);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  UA_QualifiedName cqname = UA_QUALIFIEDNAME(ns, const_cast<char *>(name.c_str()));
  rc = UA_Server_createCondition(server, UA_NODEID_NUMERIC(ns, 0), UA_NODEID_NUMERIC(0, UA_NS0ID_ALARMCONDITIONTYPE),
                                 cqname, out.source, UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), &out.node);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }

  // Add ShelvingState as an optional field. open62541's createCondition only
  // materializes mandatory fields; ShelvingState is optional in Part 9 but
  // tests assert it is present, so we add it explicitly. Failure here is
  // non-fatal - some tests skip the shelve transition path entirely.
  UA_NodeId shelving;
  UA_Server_addConditionOptionalField(server, out.node, UA_NODEID_NUMERIC(0, UA_NS0ID_ALARMCONDITIONTYPE),
                                      UA_QUALIFIEDNAME(0, const_cast<char *>("ShelvingState")), &shelving);

  // Enable the condition so events are emitted (Part 9: EnabledState=true is
  // a precondition for Retain/Active/Acked transitions to fire events).
  UA_Variant value;
  UA_Boolean enabled = true;
  UA_Variant_setScalar(&value, &enabled, &UA_TYPES[UA_TYPES_BOOLEAN]);
  rc = UA_Server_setConditionVariableFieldProperty(server, out.node, &value,
                                                   UA_QUALIFIEDNAME(0, const_cast<char *>("EnabledState")),
                                                   UA_QUALIFIEDNAME(0, const_cast<char *>("Id")));
  return rc;
}

// Add a writable Int32 variable under ObjectsFolder in the user namespace with
// a predictable string NodeId ``ns;s=<name>`` so the gateway's polled
// detection config (``node_id: "ns=2;s=StatusWord"``) can address it. Exercises
// the status_bits / fault_enum poll path that is independent of the
// AlarmConditionType event path above.
UA_StatusCode add_int32_variable(UA_Server * server, const std::string & name, UA_UInt16 ns) {
  UA_VariableAttributes attr = UA_VariableAttributes_default;
  UA_Int32 initial = 0;
  UA_Variant_setScalar(&attr.value, &initial, &UA_TYPES[UA_TYPES_INT32]);
  attr.dataType = UA_TYPES[UA_TYPES_INT32].typeId;
  attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
  attr.displayName = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>(name.c_str()));
  UA_NodeId requested = UA_NODEID_STRING_ALLOC(ns, name.c_str());
  UA_QualifiedName qname = UA_QUALIFIEDNAME(ns, const_cast<char *>(name.c_str()));
  UA_StatusCode rc = UA_Server_addVariableNode(
      server, requested, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER), UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES), qname,
      UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, nullptr, nullptr);
  UA_NodeId_clear(&requested);
  return rc;
}

// A single readable Double variable (``ns=<ns>;s=Tank.Level`` = 42.0). The
// secured integration test maps this node and reads it back over the encrypted
// SecureChannel to prove a value read - not just an event subscription -
// traverses the Basic256Sha256 channel.
UA_StatusCode add_variable(UA_Server * server, UA_UInt16 ns) {
  UA_VariableAttributes attr = UA_VariableAttributes_default;
  UA_Double value = 42.0;
  UA_Variant_setScalar(&attr.value, &value, &UA_TYPES[UA_TYPES_DOUBLE]);
  attr.displayName = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("TankLevel"));
  attr.accessLevel = UA_ACCESSLEVELMASK_READ;
  UA_NodeId node_id = UA_NODEID_STRING(ns, const_cast<char *>("Tank.Level"));
  UA_QualifiedName qname = UA_QUALIFIEDNAME(ns, const_cast<char *>("TankLevel"));
  return UA_Server_addVariableNode(server, node_id, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                   UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES), qname,
                                   UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, nullptr, nullptr);
}

UA_StatusCode set_two_state(UA_Server * server, const UA_NodeId & cond, const char * field, UA_Boolean value) {
  UA_Variant v;
  UA_Variant_setScalar(&v, &value, &UA_TYPES[UA_TYPES_BOOLEAN]);
  return UA_Server_setConditionVariableFieldProperty(server, cond, &v, UA_QUALIFIEDNAME(0, const_cast<char *>(field)),
                                                     UA_QUALIFIEDNAME(0, const_cast<char *>("Id")));
}

UA_StatusCode set_field_bool(UA_Server * server, const UA_NodeId & cond, const char * field, UA_Boolean value) {
  UA_Variant v;
  UA_Variant_setScalar(&v, &value, &UA_TYPES[UA_TYPES_BOOLEAN]);
  return UA_Server_setConditionField(server, cond, &v, UA_QUALIFIEDNAME(0, const_cast<char *>(field)));
}

UA_StatusCode handle_fire(UA_Server * server, const Condition & c, UA_UInt16 severity) {
  UA_Variant v;
  UA_Variant_setScalar(&v, &severity, &UA_TYPES[UA_TYPES_UINT16]);
  UA_StatusCode rc =
      UA_Server_setConditionField(server, c.node, &v, UA_QUALIFIEDNAME(0, const_cast<char *>("Severity")));
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  UA_LocalizedText msg = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("Alarm fired"));
  UA_Variant_setScalar(&v, &msg, &UA_TYPES[UA_TYPES_LOCALIZEDTEXT]);
  rc = UA_Server_setConditionField(server, c.node, &v, UA_QUALIFIEDNAME(0, const_cast<char *>("Message")));
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_field_bool(server, c.node, "Retain", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_two_state(server, c.node, "AckedState", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_two_state(server, c.node, "ConfirmedState", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  // ActiveState=true is the trigger that makes the A&C subsystem regenerate
  // an EventId and broadcast a CONFIRMED notification.
  rc = set_two_state(server, c.node, "ActiveState", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_clear(UA_Server * server, const Condition & c) {
  UA_StatusCode rc = set_two_state(server, c.node, "ActiveState", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_field_bool(server, c.node, "Retain", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_latch(UA_Server * server, const Condition & c) {
  UA_StatusCode rc = set_two_state(server, c.node, "ActiveState", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_field_bool(server, c.node, "Retain", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_ack(UA_Server * server, const Condition & c) {
  UA_StatusCode rc = set_two_state(server, c.node, "AckedState", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_confirm(UA_Server * server, const Condition & c) {
  UA_StatusCode rc = set_two_state(server, c.node, "ConfirmedState", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_field_bool(server, c.node, "Retain", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

// Fire a plain BaseEventType notification on the Server object (i=2253) - the
// same EventNotifier a catch-all event_alarms / auto_alarms subscribes to. A
// BaseEventType is not an AlarmConditionType, so it carries no ConditionId: the
// gateway's ConditionId select clause resolves to NodeId.Null and the poller's
// non-condition guard must drop it (it must never surface as a fault). This is
// the regression fixture for that guard - the housekeeping notifications real
// servers (e.g. a Siemens Server object) multiplex onto the alarm EventNotifier.
UA_StatusCode handle_sysevent(UA_Server * server) {
  UA_NodeId event_node = UA_NODEID_NULL;
  UA_StatusCode rc = UA_Server_createEvent(server, UA_NODEID_NUMERIC(0, UA_NS0ID_BASEEVENTTYPE), &event_node);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  UA_DateTime now = UA_DateTime_now();
  UA_Server_writeObjectProperty_scalar(server, event_node, UA_QUALIFIEDNAME(0, const_cast<char *>("Time")), &now,
                                       &UA_TYPES[UA_TYPES_DATETIME]);
  UA_UInt16 severity = 500;
  UA_Server_writeObjectProperty_scalar(server, event_node, UA_QUALIFIEDNAME(0, const_cast<char *>("Severity")),
                                       &severity, &UA_TYPES[UA_TYPES_UINT16]);
  UA_LocalizedText msg =
      UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("Session state changed to Created"));
  UA_Server_writeObjectProperty_scalar(server, event_node, UA_QUALIFIEDNAME(0, const_cast<char *>("Message")), &msg,
                                       &UA_TYPES[UA_TYPES_LOCALIZEDTEXT]);
  // originId = Server object (i=2253); delete the one-shot event node after.
  return UA_Server_triggerEvent(server, event_node, UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER), nullptr, UA_TRUE);
}

UA_StatusCode set_shelving(UA_Server * server, const Condition & c, bool shelved) {
  // ShelvingState is a ShelvedStateMachineType sub-object on the condition
  // (Part 9). open62541's experimental A&C does not implement the TimedShelve
  // / Unshelve methods, so for the test fixture we resolve the path
  // ShelvingState/CurrentState via translateBrowsePath and write both the
  // LocalizedText and the Id (NodeId) properties directly. The medkit
  // EventFilter reads ``ShelvingState/CurrentState/Id`` (a NodeId pointing
  // at one of i=2929 Unshelved / i=2930 TimedShelved / i=2932 OneShotShelved)
  // because the LocalizedText is locale-dependent. Writing only the text
  // leaves Id at its default and the bridge sees shelved=false.
  UA_RelativePathElement elems[2];
  UA_RelativePathElement_init(&elems[0]);
  UA_RelativePathElement_init(&elems[1]);
  elems[0].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("ShelvingState"));
  elems[0].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);
  elems[1].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("CurrentState"));
  elems[1].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);
  UA_BrowsePath path;
  UA_BrowsePath_init(&path);
  path.startingNode = c.node;
  path.relativePath.elementsSize = 2;
  path.relativePath.elements = elems;
  UA_BrowsePathResult result = UA_Server_translateBrowsePathToNodeIds(server, &path);
  if (result.statusCode != UA_STATUSCODE_GOOD || result.targetsSize == 0) {
    UA_BrowsePathResult_clear(&result);
    return UA_STATUSCODE_BADNOTFOUND;
  }
  UA_NodeId currentState = result.targets[0].targetId.nodeId;
  UA_LocalizedText state = shelved ? UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("TimedShelved"))
                                   : UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("Unshelved"));
  UA_Variant v;
  UA_Variant_setScalar(&v, &state, &UA_TYPES[UA_TYPES_LOCALIZEDTEXT]);
  UA_StatusCode rc = UA_Server_writeValue(server, currentState, v);
  if (rc != UA_STATUSCODE_GOOD) {
    UA_BrowsePathResult_clear(&result);
    return rc;
  }

  // Write the Id property of CurrentState. The medkit alarm bridge keys
  // suppression off ``ShelvingState/CurrentState/Id`` (one of i=2929 /
  // i=2930 / i=2932) - text is informational only.
  UA_RelativePathElement idElems[3];
  UA_RelativePathElement_init(&idElems[0]);
  UA_RelativePathElement_init(&idElems[1]);
  UA_RelativePathElement_init(&idElems[2]);
  idElems[0].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("ShelvingState"));
  idElems[0].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);
  idElems[1].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("CurrentState"));
  idElems[1].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);
  idElems[2].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("Id"));
  idElems[2].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASPROPERTY);
  UA_BrowsePath idPath;
  UA_BrowsePath_init(&idPath);
  idPath.startingNode = c.node;
  idPath.relativePath.elementsSize = 3;
  idPath.relativePath.elements = idElems;
  UA_BrowsePathResult idResult = UA_Server_translateBrowsePathToNodeIds(server, &idPath);
  if (idResult.statusCode == UA_STATUSCODE_GOOD && idResult.targetsSize > 0) {
    UA_NodeId stateIdNode = UA_NODEID_NUMERIC(0, shelved ? 2930u /* TimedShelved */ : 2929u /* Unshelved */);
    UA_Variant idVar;
    UA_Variant_setScalar(&idVar, &stateIdNode, &UA_TYPES[UA_TYPES_NODEID]);
    UA_Server_writeValue(server, idResult.targets[0].targetId.nodeId, idVar);
  }
  UA_BrowsePathResult_clear(&idResult);
  UA_BrowsePathResult_clear(&result);
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_enable(UA_Server * server, const Condition & c, bool enable) {
  return set_two_state(server, c.node, "EnabledState", enable);
}

#ifdef UA_ENABLE_ENCRYPTION

// Read a whole file into a heap-allocated UA_ByteString. Returns an empty
// ByteString (length 0) on failure; the caller treats that as a fatal config
// error. open62541's OpenSSL backend auto-detects DER (0x30 0x82 magic) vs PEM
// for both certificates and keys, so the test harness can hand us a DER cert
// and a PEM key without us caring about the encoding here.
UA_ByteString load_file(const std::string & path) {
  UA_ByteString out = UA_BYTESTRING_NULL;
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return out;
  }
  std::string data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  out.length = data.size();
  out.data = static_cast<UA_Byte *>(UA_malloc(out.length));
  if (!out.data) {
    out.length = 0;
    return out;
  }
  std::memcpy(out.data, data.data(), out.length);
  return out;
}

// Chained ActivateSession callback. open62541's UA_AccessControl_default owns
// the real user-token check; we wrap it only to emit a deterministic, greppable
// line proving which SecurityPolicy + MessageSecurityMode the SecureChannel
// actually negotiated. The test harness asserts on ``securityMode=3`` (=
// SignAndEncrypt) so the suite cannot pass against a downgraded / None channel.
using ActivateSessionFn = UA_StatusCode (*)(UA_Server *, UA_AccessControl *, const UA_EndpointDescription *,
                                            const UA_ByteString *, const UA_NodeId *, const UA_ExtensionObject *,
                                            void **);
ActivateSessionFn g_orig_activate_session = nullptr;

UA_StatusCode logging_activate_session(UA_Server * server, UA_AccessControl * ac,
                                       const UA_EndpointDescription * endpoint, const UA_ByteString * channel_cert,
                                       const UA_NodeId * session_id, const UA_ExtensionObject * user_token,
                                       void ** session_ctx) {
  std::string policy_uri;
  int mode = -1;
  if (endpoint) {
    policy_uri.assign(reinterpret_cast<const char *>(endpoint->securityPolicyUri.data),
                      endpoint->securityPolicyUri.length);
    mode = static_cast<int>(endpoint->securityMode);
  }
  std::cout << "SECURE_SESSION securityPolicyUri=" << policy_uri << " securityMode=" << mode
            << " (1=None,2=Sign,3=SignAndEncrypt)" << std::endl;
  UA_StatusCode rc = g_orig_activate_session ? g_orig_activate_session(server, ac, endpoint, channel_cert, session_id,
                                                                       user_token, session_ctx)
                                             : UA_STATUSCODE_BADINTERNALERROR;
  std::cout << "SECURE_SESSION activate rc=" << UA_StatusCode_name(rc) << std::endl;
  return rc;
}

// Build a secured server config: Basic256Sha256 with Sign AND SignAndEncrypt,
// the client app-instance cert in the trust list, and username/password access
// control with anonymous login DISABLED. We use
// UA_ServerConfig_setDefaultWithSecurityPolicies, which (per OPC UA discovery)
// also exposes a SecurityPolicy=None endpoint - open62541's connect-by-URL
// client opens a transient None channel purely to GetEndpoints, then opens the
// real encrypted channel for the session. The negotiated session security is
// proven by the SECURE_SESSION log line (securityMode=3), not by the absence of
// a None endpoint. Returns a UA status code; on success the caller adds the
// conditions exactly as in the insecure path.
UA_StatusCode configure_secure(UA_ServerConfig * config, UA_UInt16 port, const std::string & cert_path,
                               const std::string & key_path, const std::string & trust_path,
                               const std::string & app_uri, const std::string & username,
                               const std::string & password) {
  UA_ByteString cert = load_file(cert_path);
  UA_ByteString key = load_file(key_path);
  UA_ByteString trust = load_file(trust_path);
  if (cert.length == 0 || key.length == 0) {
    UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "secure: failed to read cert '%s' / key '%s'",
                 cert_path.c_str(), key_path.c_str());
    UA_ByteString_clear(&cert);
    UA_ByteString_clear(&key);
    UA_ByteString_clear(&trust);
    return UA_STATUSCODE_BADCONFIGURATIONERROR;
  }

  const UA_ByteString * trust_list = (trust.length > 0) ? &trust : nullptr;
  size_t trust_list_size = (trust.length > 0) ? 1 : 0;
  UA_StatusCode rc = UA_ServerConfig_setDefaultWithSecurityPolicies(config, port, &cert, &key, trust_list,
                                                                    trust_list_size, nullptr, 0, nullptr, 0);
  if (rc != UA_STATUSCODE_GOOD) {
    UA_ByteString_clear(&cert);
    UA_ByteString_clear(&key);
    UA_ByteString_clear(&trust);
    return rc;
  }

  // The server's applicationUri MUST equal the URI SAN in its own certificate
  // (setDefaultWithSecurityPolicies defaults it to urn:open62541.server.*).
  // open62541 stamps it into every endpoint at startup.
  UA_String_clear(&config->applicationDescription.applicationUri);
  config->applicationDescription.applicationUri = UA_STRING_ALLOC(app_uri.c_str());

  // Replace the permissive default access control (anonymous allowed) with a
  // username/password policy that rejects anonymous logins outright.
  // UA_AccessControl_default clears the previous access control itself (it
  // calls accessControl.clear at entry), so we must NOT clear it here first -
  // a manual clear double-frees the context and segfaults on open62541 1.3.
  UA_UsernamePasswordLogin login;
  login.username = UA_STRING_ALLOC(username.c_str());
  login.password = UA_STRING_ALLOC(password.c_str());
  // The user-token policy advertises Basic256Sha256 so the password is
  // encrypted at the token layer too (on top of the encrypted channel). A null
  // URI is NOT portable: open62541 1.3 dereferences it unconditionally and
  // segfaults. Non-owning UA_STRING is fine - the plugin copies it.
  UA_String token_policy_uri =
      UA_STRING(const_cast<char *>("http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256"));
  // open62541 <= 1.3 (bundled by open62541pp v0.16.0, used by the CMake fixture
  // build) takes an extra UA_CertificateVerification* (the x509 user-token
  // validator) before userTokenPolicyUri; 1.4+ (the docker image pin) dropped
  // it. We only use username/password, so pass null for it either way.
#if defined(UA_OPEN62541_VER_MAJOR) && UA_OPEN62541_VER_MAJOR == 1 && UA_OPEN62541_VER_MINOR < 4
  rc = UA_AccessControl_default(config, false /*allowAnonymous*/, nullptr /*verifyX509*/, &token_policy_uri, 1, &login);
#else
  rc = UA_AccessControl_default(config, false /*allowAnonymous*/, &token_policy_uri, 1, &login);
#endif
  UA_String_clear(&login.username);
  UA_String_clear(&login.password);
  if (rc != UA_STATUSCODE_GOOD) {
    UA_ByteString_clear(&cert);
    UA_ByteString_clear(&key);
    UA_ByteString_clear(&trust);
    return rc;
  }

  // Install the logging wrapper around the freshly-built access control.
  g_orig_activate_session = config->accessControl.activateSession;
  config->accessControl.activateSession = logging_activate_session;

  UA_ByteString_clear(&cert);
  UA_ByteString_clear(&key);
  UA_ByteString_clear(&trust);
  return rc;
}

#endif  // UA_ENABLE_ENCRYPTION

// INV2: pin explicit ServerStatus/BuildInfo so the identity integration test can
// assert known nameplate values instead of open62541's build-time defaults.
void set_build_info(UA_ServerConfig * config) {
  auto set = [](UA_String * dst, const char * value) {
    UA_String_clear(dst);
    *dst = UA_STRING_ALLOC(value);
  };
  set(&config->buildInfo.manufacturerName, "SelfPatch Test Manufacturer");
  set(&config->buildInfo.productName, "SelfPatch Test PLC");
  set(&config->buildInfo.softwareVersion, "1.2.3");
  set(&config->buildInfo.buildNumber, "build-4567");
}

// INV2: expose a minimal OPC-UA DI nameplate (Objects/DeviceSet/TestDevice with
// the standard identification properties) so the identity read exercises the DI
// path (SerialNumber / HardwareRevision) that ServerStatus/BuildInfo lacks.
// ``serial`` is overridable (--serial) so a restarted fixture can present a
// different nameplate, proving the client re-reads identity per session.
void add_di_nameplate(UA_Server * server, const std::string & serial) {
  UA_UInt16 di_ns = UA_Server_addNamespace(server, "http://opcfoundation.org/UA/DI/");

  UA_NodeId device_set_id = UA_NODEID_NULL;
  {
    UA_ObjectAttributes oa = UA_ObjectAttributes_default;
    oa.displayName = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("DeviceSet"));
    UA_Server_addObjectNode(server, UA_NODEID_NULL, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                            UA_QUALIFIEDNAME(di_ns, const_cast<char *>("DeviceSet")),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE), oa, nullptr, &device_set_id);
  }

  UA_NodeId device_id = UA_NODEID_NULL;
  {
    UA_ObjectAttributes oa = UA_ObjectAttributes_default;
    oa.displayName = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("TestDevice"));
    UA_Server_addObjectNode(server, UA_NODEID_NULL, device_set_id, UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                            UA_QUALIFIEDNAME(di_ns, const_cast<char *>("TestDevice")),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE), oa, nullptr, &device_id);
  }

  auto add_prop = [&](const char * name, const UA_DataType * type, UA_Variant value) {
    UA_VariableAttributes va = UA_VariableAttributes_default;
    va.displayName = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>(name));
    va.accessLevel = UA_ACCESSLEVELMASK_READ;
    va.dataType = type->typeId;
    va.value = value;
    UA_Server_addVariableNode(server, UA_NODEID_NULL, device_id, UA_NODEID_NUMERIC(0, UA_NS0ID_HASPROPERTY),
                              UA_QUALIFIEDNAME(di_ns, const_cast<char *>(name)),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_PROPERTYTYPE), va, nullptr, nullptr);
  };

  // Per OPC-UA DI: Manufacturer / Model are LocalizedText; SerialNumber and the
  // revisions are String.
  UA_LocalizedText manufacturer = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("SelfPatch Devices"));
  UA_Variant v_manufacturer;
  UA_Variant_setScalar(&v_manufacturer, &manufacturer, &UA_TYPES[UA_TYPES_LOCALIZEDTEXT]);
  add_prop("Manufacturer", &UA_TYPES[UA_TYPES_LOCALIZEDTEXT], v_manufacturer);

  UA_LocalizedText model = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("SPX-1000"));
  UA_Variant v_model;
  UA_Variant_setScalar(&v_model, &model, &UA_TYPES[UA_TYPES_LOCALIZEDTEXT]);
  add_prop("Model", &UA_TYPES[UA_TYPES_LOCALIZEDTEXT], v_model);

  UA_String serial_value = UA_STRING(const_cast<char *>(serial.c_str()));
  UA_Variant v_serial;
  UA_Variant_setScalar(&v_serial, &serial_value, &UA_TYPES[UA_TYPES_STRING]);
  add_prop("SerialNumber", &UA_TYPES[UA_TYPES_STRING], v_serial);

  UA_String hardware = UA_STRING(const_cast<char *>("HW-A2"));
  UA_Variant v_hardware;
  UA_Variant_setScalar(&v_hardware, &hardware, &UA_TYPES[UA_TYPES_STRING]);
  add_prop("HardwareRevision", &UA_TYPES[UA_TYPES_STRING], v_hardware);

  UA_String software = UA_STRING(const_cast<char *>("SW-3.4.5"));
  UA_Variant v_software;
  UA_Variant_setScalar(&v_software, &software, &UA_TYPES[UA_TYPES_STRING]);
  add_prop("SoftwareRevision", &UA_TYPES[UA_TYPES_STRING], v_software);

  // Vendor extension: OrderNumber (AAS ManufacturerOrderCode) exposed under a
  // vendor namespace (NOT the DI namespace) as the real Siemens S7-1500 MLFB
  // (ns=3;s=OrderNumber), padded with leading AND trailing spaces. The internal
  // space in "6ES7 672-..." is load-bearing and must survive the edge trim, so a
  // whitespace-collapsing reimplementation of trim_ascii_ws would fail the E2E
  // assertion. Exercises match-by-BrowseName-across-namespaces + whitespace-trim.
  UA_UInt16 vendor_ns = UA_Server_addNamespace(server, "http://selfpatch.test/vendor");
  UA_VariableAttributes va_order = UA_VariableAttributes_default;
  va_order.displayName = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("OrderNumber"));
  va_order.accessLevel = UA_ACCESSLEVELMASK_READ;
  va_order.dataType = UA_TYPES[UA_TYPES_STRING].typeId;
  UA_String order = UA_STRING(const_cast<char *>("  6ES7 672-5SC11-0YA0  "));
  UA_Variant_setScalar(&va_order.value, &order, &UA_TYPES[UA_TYPES_STRING]);
  UA_Server_addVariableNode(server, UA_NODEID_NULL, device_id, UA_NODEID_NUMERIC(0, UA_NS0ID_HASPROPERTY),
                            UA_QUALIFIEDNAME(vendor_ns, const_cast<char *>("OrderNumber")),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_PROPERTYTYPE), va_order, nullptr, nullptr);
}

void cli_loop(UA_Server * server, UA_UInt16 ns) {
  std::string line;
  while (g_running && std::getline(std::cin, line)) {
    std::istringstream iss(line);
    std::string cmd, name;
    iss >> cmd >> name;
    if (cmd == "quit") {
      g_running = false;
      std::cout << "OK quit" << std::endl;
      break;
    }
    std::lock_guard<std::mutex> guard(g_mutex);
    // ``set <NodeName> <int>`` writes a polled Int32 variable (StatusWord /
    // FaultCode). Handled before the condition lookup because these nodes are
    // plain variables, not AlarmCondition instances in g_conditions.
    if (cmd == "set") {
      long val = 0;
      if (!(iss >> val)) {
        std::cout << "ERR set_missing_value:" << name << std::endl;
        continue;
      }
      UA_Int32 v32 = static_cast<UA_Int32>(val);
      UA_Variant var;
      UA_Variant_setScalar(&var, &v32, &UA_TYPES[UA_TYPES_INT32]);
      UA_NodeId target = UA_NODEID_STRING_ALLOC(ns, name.c_str());
      UA_StatusCode rc = UA_Server_writeValue(server, target, var);
      UA_NodeId_clear(&target);
      if (rc == UA_STATUSCODE_GOOD) {
        std::cout << "OK " << name << "=" << val << std::endl;
      } else {
        std::cout << "ERR " << name << ":" << UA_StatusCode_name(rc) << std::endl;
      }
      continue;
    }
    // ``sysevent`` fires a non-condition BaseEventType on the Server object
    // (i=2253); it has no <name> and is not in g_conditions, so handle it
    // before the condition lookup (like ``quit`` / ``set``).
    if (cmd == "sysevent") {
      UA_StatusCode rc = handle_sysevent(server);
      if (rc == UA_STATUSCODE_GOOD) {
        std::cout << "OK sysevent" << std::endl;
      } else {
        std::cout << "ERR sysevent:" << UA_StatusCode_name(rc) << std::endl;
      }
      continue;
    }
    auto it = g_conditions.find(name);
    if (cmd != "quit" && it == g_conditions.end()) {
      std::cout << "ERR unknown_condition:" << name << std::endl;
      continue;
    }
    Condition & cref = it->second;
    UA_StatusCode rc = UA_STATUSCODE_BADNOTSUPPORTED;
    if (cmd == "fire") {
      UA_UInt16 sev = 500;
      iss >> sev;
      rc = handle_fire(server, cref, sev);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.active = true;
        cref.acked = false;
        cref.confirmed = false;
        cref.retain = true;
      }
    } else if (cmd == "clear") {
      rc = handle_clear(server, cref);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.active = false;
        cref.retain = false;
      }
    } else if (cmd == "latch") {
      rc = handle_latch(server, cref);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.active = false;
        cref.retain = true;
      }
    } else if (cmd == "ack") {
      rc = handle_ack(server, cref);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.acked = true;
      }
    } else if (cmd == "confirm") {
      rc = handle_confirm(server, cref);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.confirmed = true;
        cref.retain = false;
      }
    } else if (cmd == "shelve") {
      rc = set_shelving(server, cref, true);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.shelved = true;
      }
    } else if (cmd == "unshelve") {
      rc = set_shelving(server, cref, false);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.shelved = false;
      }
    } else if (cmd == "disable") {
      rc = handle_enable(server, cref, false);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.enabled = false;
      }
    } else if (cmd == "enable") {
      rc = handle_enable(server, cref, true);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.enabled = true;
      }
    } else {
      std::cout << "ERR unknown_cmd:" << cmd << std::endl;
      continue;
    }
    if (rc == UA_STATUSCODE_GOOD) {
      std::cout << "OK " << name << std::endl;
      log_state(cref);
    } else {
      std::cout << "ERR " << name << ":" << UA_StatusCode_name(rc) << std::endl;
    }
  }
}

}  // namespace

int main(int argc, char ** argv) {
  signal(SIGINT, stop_handler);
  signal(SIGTERM, stop_handler);

  UA_UInt16 port = 4842;
  bool secure = false;
  std::string cert_path, key_path, trust_path, username = "medkit", password = "secret";
  std::string app_uri = "urn:test:alarms:server";
  std::string di_serial = "SN-0001-TEST";
  UA_UInt32 max_refs_per_node = 0;  // 0 = server default (unlimited)
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = static_cast<UA_UInt16>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--serial") == 0 && i + 1 < argc) {
      di_serial = argv[++i];
    } else if (std::strcmp(argv[i], "--max-refs-per-node") == 0 && i + 1 < argc) {
      // Caps references per Browse result so every larger browse pages via
      // BrowseNext continuation points (regression fixture for the client's
      // continuation-point handling).
      max_refs_per_node = static_cast<UA_UInt32>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--secure") == 0) {
      secure = true;
    } else if (std::strcmp(argv[i], "--cert") == 0 && i + 1 < argc) {
      cert_path = argv[++i];
    } else if (std::strcmp(argv[i], "--key") == 0 && i + 1 < argc) {
      key_path = argv[++i];
    } else if (std::strcmp(argv[i], "--trust") == 0 && i + 1 < argc) {
      trust_path = argv[++i];
    } else if (std::strcmp(argv[i], "--app-uri") == 0 && i + 1 < argc) {
      app_uri = argv[++i];
    } else if (std::strcmp(argv[i], "--username") == 0 && i + 1 < argc) {
      username = argv[++i];
    } else if (std::strcmp(argv[i], "--password") == 0 && i + 1 < argc) {
      password = argv[++i];
    }
  }

  UA_Server * server = UA_Server_new();
  UA_ServerConfig * config = UA_Server_getConfig(server);
  if (secure) {
#ifdef UA_ENABLE_ENCRYPTION
    UA_StatusCode src = configure_secure(config, port, cert_path, key_path, trust_path, app_uri, username, password);
    if (src != UA_STATUSCODE_GOOD) {
      UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "secure config failed: %s", UA_StatusCode_name(src));
      UA_Server_delete(server);
      return 1;
    }
#else
    UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND,
                 "--secure requested but this build has no UA_ENABLE_ENCRYPTION support");
    UA_Server_delete(server);
    return 1;
#endif
  } else {
    UA_ServerConfig_setMinimal(config, port, nullptr);
  }
  set_build_info(config);

  UA_UInt16 ns = UA_Server_addNamespace(server, NS_URI);
  if (add_variable(server, ns) != UA_STATUSCODE_GOOD) {
    UA_LOG_WARNING(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Failed to register Tank.Level variable");
  }

  // INV2: standard device-info nameplate for the identity integration test.
  add_di_nameplate(server, di_serial);

  Condition op, oh, sl;
  if (add_condition(server, "Overpressure", ns, op) != UA_STATUSCODE_GOOD ||
      add_condition(server, "Overheat", ns, oh) != UA_STATUSCODE_GOOD ||
      add_condition(server, "SensorLost", ns, sl) != UA_STATUSCODE_GOOD) {
    UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Failed to register conditions");
    UA_Server_delete(server);
    return 1;
  }
  g_conditions[op.name] = op;
  g_conditions[oh.name] = oh;
  g_conditions[sl.name] = sl;

  // Polled Int32 registers for the status_bits / fault_enum detection path
  // (independent of the AlarmConditionType events above). Driven via the
  // ``set <NodeName> <int>`` CLI command.
  if (add_int32_variable(server, "StatusWord", ns) != UA_STATUSCODE_GOOD ||
      add_int32_variable(server, "FaultCode", ns) != UA_STATUSCODE_GOOD) {
    UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Failed to register detection variables");
    UA_Server_delete(server);
    return 1;
  }

  // Apply the browse cap only AFTER all nodes exist: open62541's own AddNode /
  // createCondition machinery browses internally and fails with
  // BadNoContinuationPoints when the cap is active during setup.
  if (max_refs_per_node > 0) {
    config->maxReferencesPerNode = max_refs_per_node;
  }

  std::cout << "READY port=" << port << " namespace=" << ns << " secure=" << (secure ? "true" : "false") << std::endl;
  std::thread cli(cli_loop, server, ns);

  UA_StatusCode rc = UA_Server_run(server, reinterpret_cast<volatile UA_Boolean *>(&g_running));
  g_running = false;
  // The exit code is 1 for every bad status, so print the status itself:
  // otherwise a server that ends on its own leaves the driving test with a
  // closed stdin pipe and no reason anywhere.
  if (rc != UA_STATUSCODE_GOOD) {
    std::cout << "EXIT UA_Server_run rc=" << UA_StatusCode_name(rc) << std::endl;
  }
  if (cli.joinable()) {
    cli.join();
  }
  UA_Server_delete(server);
  return rc == UA_STATUSCODE_GOOD ? 0 : 1;
}
