# Copyright 2026 mfaferek93
#
# Standalone OPC-UA test fixture image for AlarmConditionType integration
# tests (issue #386). The fixture exposes 3 conditions on tcp port 4842 and
# reads CLI commands from stdin; see ../../test/fixtures/test_alarm_server/.
#
# Build context: ros2_medkit repo root.
#   docker build --network=host \
#                -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
#                -t ros2_medkit_alarm_test_server:dev .

FROM ubuntu:24.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
# apt-get update fetches package indexes from the distribution mirrors, a
# network read that a reset connection fails outright and that runs before
# anything of ours is built. Three attempts, five seconds apart; each
# attempt's own error text goes to stderr, so the last failure is what a
# reader sees.
RUN retry() { n=1; while :; do "$@" && return 0; [ "$n" -ge 3 ] && return 1; echo "attempt $n of 3 failed: $*" >&2; n=$((n+1)); sleep 5; done; }; \
    retry apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git python3 ca-certificates libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Pin matches the SHA used by the plugin's open62541pp dependency
# (122ea4c842 == open62541pp v0.16.0). open62541 itself is a submodule of
# open62541pp; we only need the inner repo for the standalone server build.
ARG OPEN62541_REF=v1.4.6
WORKDIR /src
RUN git clone --depth=1 --branch=${OPEN62541_REF} \
        https://github.com/open62541/open62541.git /src/open62541 \
    && cd /src/open62541 && git submodule update --init --recursive --depth=1

COPY src/ros2_medkit_plugins/ros2_medkit_opcua/test/fixtures/test_alarm_server \
     /src/test_alarm_server

RUN cmake -S /src/open62541 -B /tmp/o62-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/open62541 \
        -DBUILD_SHARED_LIBS=OFF \
        -DUA_ENABLE_SUBSCRIPTIONS_EVENTS=ON \
        -DUA_ENABLE_SUBSCRIPTIONS_ALARMS_CONDITIONS=ON \
        -DUA_ENABLE_ENCRYPTION=OPENSSL \
        -DUA_NAMESPACE_ZERO=FULL \
        -DUA_BUILD_EXAMPLES=OFF \
        -DUA_BUILD_TOOLS=OFF \
        -DUA_FORCE_WERROR=OFF \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_C_FLAGS=-w \
    && cmake --build /tmp/o62-build -j"$(nproc)" --target install

# The static lib is built with the OpenSSL encryption backend, so link OpenSSL.
RUN g++ -O2 -std=c++17 -w \
        -I/opt/open62541/include \
        /src/test_alarm_server/test_alarm_server.cpp \
        /opt/open62541/lib/libopen62541.a \
        -lssl -lcrypto -lpthread -o /opt/test_alarm_server

FROM ubuntu:24.04
# Same retry as above, same reason.
RUN retry() { n=1; while :; do "$@" && return 0; [ "$n" -ge 3 ] && return 1; echo "attempt $n of 3 failed: $*" >&2; n=$((n+1)); sleep 5; done; }; \
    retry apt-get update && apt-get install -y --no-install-recommends \
        libstdc++6 ca-certificates libssl3 openssl \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /opt/test_alarm_server /usr/local/bin/test_alarm_server
EXPOSE 4842
ENTRYPOINT ["/usr/local/bin/test_alarm_server"]
CMD ["--port", "4842"]
