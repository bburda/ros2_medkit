# OpenPLC Runtime v4 (Autonomy-Logic) with tank demo
# Strategy: Build runtime, then at startup use the web API to upload,
# compile and start the ST program through OpenPLC's full pipeline.

FROM debian:bookworm-slim AS matiec-builder

# apt-get update fetches package indexes from the distribution mirrors, a
# network read that a reset connection fails outright and that runs before
# anything of ours is built. Three attempts, five seconds apart; each
# attempt's own error text goes to stderr, so the last failure is what a
# reader sees.
RUN retry() { n=1; while :; do "$@" && return 0; [ "$n" -ge 3 ] && return 1; echo "attempt $n of 3 failed: $*" >&2; n=$((n+1)); sleep 5; done; }; \
    retry apt-get update && apt-get install -y \
    git build-essential flex bison autoconf automake \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/beremiz/matiec.git /matiec && \
    cd /matiec && autoreconf -i && ./configure && make -j$(nproc) && make install


FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /openplc

# Pin to specific commit for reproducible builds
ENV OPENPLC_COMMIT=8a22c81ed6ddaba13225caec4a3ff15fd2c92909
# Same retry as above, same reason.
RUN retry() { n=1; while :; do "$@" && return 0; [ "$n" -ge 3 ] && return 1; echo "attempt $n of 3 failed: $*" >&2; n=$((n+1)); sleep 5; done; }; \
    retry apt-get update && apt-get install -y git ca-certificates curl jq && \
    git clone https://github.com/Autonomy-Logic/openplc-runtime.git /openplc && \
    cd /openplc && git checkout ${OPENPLC_COMMIT} && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /var/run/runtime && \
    chmod +x install.sh scripts/* start_openplc.sh && \
    rm -rf build/ venvs/ .venv/ 2>/dev/null || true && \
    ./install.sh && \
    rm -rf /var/lib/apt/lists/*

# Enable OPC-UA plugin and install config
RUN sed -i 's/^opcua,\(.*\),0,0,/opcua,\1,1,0,/' plugins.conf && \
    grep -q '^opcua,.*,1,' plugins.conf || \
    (echo "ERROR: Failed to enable OPC-UA plugin in plugins.conf" && exit 1)
COPY opcua_config.json /openplc/core/src/drivers/plugins/python/opcua/opcua.json

# Copy matiec for ST -> C compilation
COPY --from=matiec-builder /usr/local/bin/iec2c /usr/local/bin/iec2c
COPY --from=matiec-builder /matiec/lib /matiec/lib

# Copy tank demo ST program
COPY tank_demo.st /openplc/programs/tank_demo.st

# Pre-compile ST -> C and place in core/generated with proper IEC lib
RUN mkdir -p /openplc/core/generated && \
    iec2c -I /matiec/lib -T /openplc/core/generated /openplc/programs/tank_demo.st && \
    cp -r /matiec/lib/C/* /openplc/core/generated/lib/ 2>/dev/null; \
    mkdir -p /openplc/core/generated/lib && \
    cp -r /matiec/lib/C/* /openplc/core/generated/lib/

# Copy hand-written glueVars that implements all required runtime symbols
COPY glueVars_tank.c /openplc/core/generated/glueVars.c

# debug.c and c_blocks_code stubs
COPY debug_stub.c /openplc/core/generated/debug.c
RUN echo '// no custom C blocks' > /openplc/core/generated/c_blocks_code.cpp

# Entrypoint: compile PLC program then start runtime
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 8443 4840

CMD ["/entrypoint.sh"]
