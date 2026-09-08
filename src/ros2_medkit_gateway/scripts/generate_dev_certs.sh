#!/bin/bash
# Copyright 2025 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Generate self-signed development certificates for TLS/HTTPS testing
#
# WARNING: These certificates are for DEVELOPMENT ONLY.
# Do NOT use in production environments.
#
# Usage:
#   ./generate_dev_certs.sh [output_dir]
#
# Default output directory: ./certs
#
# Generated files:
#   - cert.pem: Self-signed certificate
#   - key.pem: Private key (RSA 4096-bit)
#   - ca.pem: CA certificate

set -e

# Check for required tools
command -v openssl >/dev/null 2>&1 || {
  echo "Error: openssl is required but not installed."
  echo "Install it with: sudo apt-get install openssl"
  exit 1
}

# Output directory
OUTPUT_DIR="${1:-./certs}"

# Certificate validity (days)
VALIDITY_DAYS=365

# Key size
KEY_SIZE=4096

# Common name for server certificate
SERVER_CN="${SERVER_CN:-localhost}"

# Subject for certificates
SUBJECT="/C=US/ST=Development/L=Local/O=ROS2Medkit/OU=Gateway"

echo "=== ROS 2 Medkit Gateway - Development Certificate Generator ==="
echo "WARNING: These certificates are for DEVELOPMENT ONLY."
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"
cd "$OUTPUT_DIR"

echo "Generating certificates in: $(pwd)"
echo ""

# Generate CA certificate
echo "[1/3] Generating CA certificate..."
openssl req -x509 -newkey rsa:$KEY_SIZE -keyout ca_key.pem -out ca.pem \
    -days $VALIDITY_DAYS -nodes \
    -subj "$SUBJECT/CN=ROS2Medkit Development CA" \
    2>/dev/null

# Generate server private key
echo "[2/3] Generating server private key..."
openssl genrsa -out key.pem $KEY_SIZE 2>/dev/null

# Generate server certificate signing request
echo "[3/3] Generating server certificate..."
openssl req -new -key key.pem -out server.csr \
    -subj "$SUBJECT/CN=$SERVER_CN" \
    2>/dev/null

# Create extension file for SAN (Subject Alternative Names)
cat > server_ext.cnf << EOF
authorityKeyIdentifier=keyid,issuer
basicConstraints=CA:FALSE
keyUsage = digitalSignature, nonRepudiation, keyEncipherment, dataEncipherment
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
DNS.2 = $SERVER_CN
IP.1 = 127.0.0.1
IP.2 = ::1
EOF

# Sign server certificate with CA
openssl x509 -req -in server.csr -CA ca.pem -CAkey ca_key.pem -CAcreateserial \
    -out cert.pem -days $VALIDITY_DAYS \
    -extfile server_ext.cnf \
    2>/dev/null

# Cleanup temporary files
rm -f server.csr server_ext.cnf ca.srl

# Set secure permissions on private keys
chmod 600 key.pem ca_key.pem

echo ""
echo "=== Certificate Generation Complete ==="
echo ""
echo "Generated files:"
echo "  Server certificate: $OUTPUT_DIR/cert.pem"
echo "  Server private key: $OUTPUT_DIR/key.pem"
echo "  CA certificate:     $OUTPUT_DIR/ca.pem"
echo "  CA private key:     $OUTPUT_DIR/ca_key.pem"
echo ""
echo "Example gateway configuration (gateway_params.yaml):"
echo ""
echo "  server:"
echo "    tls:"
echo "      enabled: true"
echo "      cert_file: \"$OUTPUT_DIR/cert.pem\""
echo "      key_file: \"$OUTPUT_DIR/key.pem\""
echo ""
echo "  Do NOT add ca_file here. It is not the CA a client verifies the server"
echo "  with - setting it makes the gateway REQUIRE a client certificate from"
echo "  every caller, and the curl below would then be refused. Pass ca.pem to"
echo "  the client with --cacert instead, as shown."
echo ""
echo "Test with curl:"
echo "  curl -k https://localhost:8080/api/v1/health"
echo ""
echo "Test with curl (verify certificate):"
echo "  curl --cacert $OUTPUT_DIR/ca.pem https://localhost:8080/api/v1/health"
