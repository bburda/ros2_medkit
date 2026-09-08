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

"""
Launch file for ROS 2 Medkit Gateway with HTTPS/TLS enabled.

This launch file automatically generates self-signed development certificates
and starts the gateway with TLS enabled. For production deployments, use
proper CA-signed certificates and the standard gateway.launch.py.

Usage:
    ros2 launch ros2_medkit_gateway gateway_https.launch.py

    # With custom port:
    ros2 launch ros2_medkit_gateway gateway_https.launch.py server_port:=8443

    # Persist certificates to custom directory:
    ros2 launch ros2_medkit_gateway gateway_https.launch.py cert_dir:=/path/to/certs

WARNING: Generated certificates are for DEVELOPMENT ONLY.
         Do NOT use in production environments.
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Default web UI origins enabled when the user does not override CORS, so the
# bundled web UI works over HTTPS out of the box (kept in sync with
# gateway.launch.py; a wildcard is deliberately not used).
CORS_DEFAULT = 'http://localhost:3000,http://localhost:5173'


def cors_override(cors_arg, config_file, default_config):
    """
    Return the ``cors.allowed_origins`` entry for the final overrides, or {}.

    The final overrides dict wins per key, so to avoid silently overriding a
    user's ``config_file``: an explicit ``cors_allowed_origins`` arg always wins
    (empty -> [''], CORS off); with no arg and a custom config file, inject
    nothing so the file's value is respected; with no arg and the default config,
    apply the web UI origins so the bundled UI works out of the box.
    """
    if cors_arg.strip() != CORS_DEFAULT:
        origins = [o.strip() for o in cors_arg.split(',') if o.strip()] or ['']
        return {'cors.allowed_origins': origins}
    if not config_file or config_file == default_config:
        return {'cors.allowed_origins': CORS_DEFAULT.split(',')}
    return {}


def generate_certificates(cert_dir: str) -> dict:
    """
    Generate self-signed certificates for TLS/HTTPS.

    Parameters
    ----------
    cert_dir : str
        Directory to store generated certificates

    Returns
    -------
    dict
        Paths to generated certificate files

    """
    cert_file = os.path.join(cert_dir, 'cert.pem')
    key_file = os.path.join(cert_dir, 'key.pem')
    ca_file = os.path.join(cert_dir, 'ca.pem')

    # Skip if certificates already exist
    if os.path.exists(cert_file) and os.path.exists(key_file):
        return {
            'cert_file': cert_file,
            'key_file': key_file,
            # Deliberately NOT passed as server.tls.ca_file. This CA signs the
            # SERVER certificate so a client can verify the gateway; setting it
            # as the gateway's ca_file turns on mutual TLS and rejects every
            # client that has no certificate of its own, including the curl
            # this launch file prints. Kept here only so the hint below can
            # tell the user which CA to pass with --cacert.
            'ca_file_for_client': ca_file if os.path.exists(ca_file) else '',
        }

    os.makedirs(cert_dir, exist_ok=True)

    # Generate CA key and certificate
    ca_key_file = os.path.join(cert_dir, 'ca_key.pem')
    subprocess.run([
        'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
        '-keyout', ca_key_file,
        '-out', ca_file,
        '-days', '30', '-nodes',
        '-subj', '/C=US/ST=Dev/L=Local/O=ROS2Medkit/CN=DevCA'
    ], check=True, capture_output=True)

    # Generate server key
    subprocess.run([
        'openssl', 'genrsa', '-out', key_file, '2048'
    ], check=True, capture_output=True)

    # Generate CSR
    csr_file = os.path.join(cert_dir, 'server.csr')
    subprocess.run([
        'openssl', 'req', '-new', '-key', key_file, '-out', csr_file,
        '-subj', '/C=US/ST=Dev/L=Local/O=ROS2Medkit/CN=localhost'
    ], check=True, capture_output=True)

    # Create extension file for SAN
    ext_file = os.path.join(cert_dir, 'ext.cnf')
    with open(ext_file, 'w') as f:
        f.write("""basicConstraints=CA:FALSE
keyUsage = digitalSignature, keyEncipherment
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
IP.1 = 127.0.0.1
IP.2 = ::1
""")

    # Sign certificate with CA
    subprocess.run([
        'openssl', 'x509', '-req', '-in', csr_file,
        '-CA', ca_file,
        '-CAkey', ca_key_file,
        '-CAcreateserial',
        '-out', cert_file,
        '-days', '30',
        '-extfile', ext_file
    ], check=True, capture_output=True)

    # Cleanup temporary files
    for tmp_file in [csr_file, ext_file, os.path.join(cert_dir, 'ca.srl')]:
        if os.path.exists(tmp_file):
            os.remove(tmp_file)

    # Set secure permissions on private keys
    os.chmod(key_file, 0o600)
    os.chmod(ca_key_file, 0o600)

    return {
        'cert_file': cert_file,
        'key_file': key_file,
        # See the note above: this is the CA a CLIENT verifies the server with,
        # not a client-certificate authority for the gateway to demand.
        'ca_file_for_client': ca_file,
    }


def launch_setup(context):
    """Set up launch with generated certificates."""
    # Get launch configuration values
    cert_dir = LaunchConfiguration('cert_dir').perform(context)
    server_host = LaunchConfiguration('server_host').perform(context)
    server_port = LaunchConfiguration('server_port').perform(context)
    refresh_interval = LaunchConfiguration('refresh_interval_ms').perform(context)
    min_tls_version = LaunchConfiguration('min_tls_version').perform(context)
    override_config = LaunchConfiguration('config_file').perform(context)
    cors_arg = LaunchConfiguration('cors_allowed_origins').perform(context)

    # Use temp directory if not specified
    if not cert_dir:
        cert_dir = os.path.join(tempfile.gettempdir(), 'ros2_medkit_certs')

    # Generate certificates
    cert_paths = generate_certificates(cert_dir)

    # Get package directory for base config
    pkg_dir = get_package_share_directory('ros2_medkit_gateway')
    default_config = os.path.join(pkg_dir, 'config', 'gateway_params.yaml')

    # Protocol info for logging
    protocol = 'HTTPS' if cert_paths['cert_file'] else 'HTTP'

    return [
        LogInfo(msg=['='*60]),
        LogInfo(msg=['ROS 2 Medkit Gateway - HTTPS/TLS Mode']),
        LogInfo(msg=['='*60]),
        LogInfo(msg=['WARNING: Using self-signed development certificates!']),
        LogInfo(msg=['         Do NOT use in production.']),
        LogInfo(msg=['']),
        LogInfo(msg=['Certificate directory: ', cert_dir]),
        LogInfo(msg=['Server URL: ', f'{protocol.lower()}://{server_host}:{server_port}']),
        LogInfo(msg=['']),
        LogInfo(msg=['Test with curl:']),
        LogInfo(msg=[f'  curl -k https://{server_host}:{server_port}/api/v1/health']),
        LogInfo(msg=['']),
        LogInfo(msg=['Test with CA verification:']),
        LogInfo(msg=[f'  curl --cacert {cert_paths["ca_file_for_client"]} '
                     f'https://{server_host}:{server_port}/api/v1/health']),
        LogInfo(msg=['='*60]),

        Node(
            package='ros2_medkit_gateway',
            executable='gateway_node',
            name='ros2_medkit_gateway',
            output='screen',
            parameters=[
                default_config, override_config,
                {
                    'server.host': server_host,
                    'server.port': int(server_port),
                    'server.tls.enabled': True,
                    'server.tls.cert_file': cert_paths['cert_file'],
                    'server.tls.key_file': cert_paths['key_file'],
                    'server.tls.min_version': min_tls_version,
                    'refresh_interval_ms': int(refresh_interval),
                    **cors_override(cors_arg, override_config, default_config),
                }
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ]


def generate_launch_description():
    """Generate launch description for HTTPS gateway."""
    return LaunchDescription([
        # Certificate directory (empty = use temp)
        DeclareLaunchArgument(
            'cert_dir',
            default_value='',
            description='Directory for certificates (empty = temp dir)'
        ),

        # Server configuration
        DeclareLaunchArgument(
            'server_host',
            default_value='127.0.0.1',
            description='Host to bind HTTPS server (127.0.0.1 or 0.0.0.0)'
        ),

        DeclareLaunchArgument(
            'server_port',
            default_value='8443',
            description='Port for HTTPS API (default 8443 to avoid conflict with HTTP)'
        ),

        DeclareLaunchArgument(
            'refresh_interval_ms',
            default_value='30000',
            description=(
                'Safety-backstop refresh interval in milliseconds. Primary '
                'refresh is graph-event driven; this only controls the '
                'periodic forced refresh. Must match the default in '
                'config/gateway_params.yaml.'
            )
        ),

        DeclareLaunchArgument(
            'min_tls_version',
            default_value='1.2',
            description='Minimum TLS version (1.2 or 1.3)'
        ),

        DeclareLaunchArgument(
            'config_file', default_value=os.path.join(
                get_package_share_directory('ros2_medkit_gateway'),
                'config', 'gateway_params.yaml'),
            description='Path to YAML config file to override gateway parameters. Default config '
                        'is the ros2_medkit_gateway/config/gateway_params.yaml.'
        ),

        DeclareLaunchArgument(
            'cors_allowed_origins',
            default_value=CORS_DEFAULT,
            description='Comma-separated CORS origins allowed to call the gateway from a browser, '
                        'so the web UI works over HTTPS out of the box. Pass an explicit value to '
                        'override (empty disables CORS); when left at the default, a config_file '
                        'that sets cors.allowed_origins is respected.'
        ),

        # Use OpaqueFunction to generate certs and set up node
        OpaqueFunction(function=launch_setup),
    ])
