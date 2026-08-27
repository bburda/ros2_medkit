# Copyright 2025 mfaferek93
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

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Default web UI origins enabled when the user does not override CORS, so the
# bundled web UI works out of the box. A wildcard is deliberately not used.
CORS_DEFAULT = 'http://localhost:3000,http://localhost:5173'


def cors_override(cors_arg, config_file, default_config):
    """
    Return the ``cors.allowed_origins`` entry for the final overrides, or {}.

    The final overrides dict is applied after the config file, so anything in it
    wins per key. To avoid silently overriding a user's ``config_file``:

    - an explicit ``cors_allowed_origins`` arg always wins (empty -> [''], which
      the gateway reads as CORS off);
    - with no arg and a *custom* config file, inject nothing so the file's
      ``cors.allowed_origins`` is respected;
    - with no arg and the default config, apply the web UI origins so the bundled
      UI works out of the box.

    An empty list cannot be passed as a launch parameter (and is the untyped-empty
    shape that aborts startup), so the off case uses [''] - the same placeholder
    the gateway config ships; config.cpp filters the empty string.
    """
    if cors_arg.strip() != CORS_DEFAULT:
        origins = [o.strip() for o in cors_arg.split(',') if o.strip()] or ['']
        return {'cors.allowed_origins': origins}
    if not config_file or config_file == default_config:
        return {'cors.allowed_origins': CORS_DEFAULT.split(',')}
    return {}


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_medkit_gateway')
    default_config = os.path.join(pkg_dir, 'config', 'gateway_params.yaml')

    # Resolve graph provider plugin path (optional - gateway starts without it)
    graph_provider_path = ''
    try:
        graph_provider_prefix = get_package_prefix('ros2_medkit_graph_provider')
        graph_provider_path = os.path.join(
            graph_provider_prefix, 'lib', 'ros2_medkit_graph_provider',
            'libros2_medkit_graph_provider_plugin.so')
    except PackageNotFoundError:
        print('[gateway.launch.py] ros2_medkit_graph_provider not installed '
              '- graph endpoints will not be available')

    if graph_provider_path and not os.path.isfile(graph_provider_path):
        print('[gateway.launch.py] WARNING: graph provider .so not found at '
              f'{graph_provider_path} - plugin will not load')
        graph_provider_path = ''

    declare_override_config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to YAML config file to override gateway parameters. Default config '
                    'is the ros2_medkit_gateway/config/gateway_params.yaml.')

    declare_host_arg = DeclareLaunchArgument(
        'server_host', default_value='127.0.0.1',
        description='Host to bind REST server (127.0.0.1 or 0.0.0.0)')

    declare_port_arg = DeclareLaunchArgument(
        'server_port', default_value='8080',
        description='Port for REST API')

    declare_refresh_arg = DeclareLaunchArgument(
        'refresh_interval_ms', default_value='30000',
        description=(
            'Safety-backstop refresh interval in milliseconds. Primary '
            'refresh is graph-event driven (~100 ms latency); this only '
            'controls the periodic forced refresh. Must match the default '
            'in config/gateway_params.yaml.'))

    declare_jwt_secret_arg = DeclareLaunchArgument(
        'jwt_secret', default_value='',
        description=(
            'HS256 signing secret, at least 32 characters. REQUIRED: the '
            'shipped config has auth.enabled true, and the gateway refuses to '
            'start without a secret. Pass one here, or point config_file at a '
            'file that sets auth.jwt_secret and auth.clients. To run without '
            'authentication - only ever on a host nothing else can reach - '
            'pass auth_enabled:=false explicitly.'))

    declare_auth_enabled_arg = DeclareLaunchArgument(
        'auth_enabled', default_value='true',
        description=(
            'Require a credential. On by default, matching the shipped '
            'config. Turning it off makes the entity tree, the fault history '
            'and every operation readable by anyone who can reach the port.'))

    declare_clients_arg = DeclareLaunchArgument(
        'auth_clients', default_value='',
        description=(
            'Comma-separated "client_id:client_secret:role" triples '
            '(roles: viewer, operator, configurator, admin). Needed to obtain '
            'a token from /auth/token.'))

    declare_tls_enabled_arg = DeclareLaunchArgument(
        'tls_enabled', default_value='true',
        description=(
            'Serve HTTPS. On by default, matching the shipped config. Needs '
            'cert_file and key_file; the gateway refuses to start with TLS on '
            'and no certificate rather than fall back to plaintext. Pass '
            'tls_enabled:=false to serve plain HTTP on a host nothing else '
            'can reach.'))

    declare_cert_file_arg = DeclareLaunchArgument(
        'cert_file', default_value='',
        description=(
            'PEM certificate (or full chain) for HTTPS. REQUIRED while '
            'tls_enabled is true. For a first run, generate a self-signed '
            'pair with scripts/generate_dev_certs.sh - browsers will warn, '
            'which is correct for a certificate nothing has vouched for.'))

    declare_key_file_arg = DeclareLaunchArgument(
        'key_file', default_value='',
        description=(
            'PEM private key matching cert_file. REQUIRED while tls_enabled '
            'is true. Keep it chmod 600 and owned by the gateway user.'))

    declare_cors_arg = DeclareLaunchArgument(
        'cors_allowed_origins',
        default_value=CORS_DEFAULT,
        description='Comma-separated CORS origins allowed to call the REST API from a browser, so '
                    'the bundled web UI (a different origin) works out of the box. Pass an '
                    'explicit value to override (empty disables CORS); when left at the default, '
                    'a config_file that sets cors.allowed_origins is respected. A wildcard is '
                    'intentionally not the default: with auth off and write methods enabled it '
                    'would let any site drive cross-origin writes.')

    # The cors arg is resolved at launch time (a comma-separated LaunchConfiguration
    # cannot be passed straight through as a string-array parameter) and folded in
    # by cors_override, which keeps a config_file's CORS from being silently
    # overridden by the launch default.
    def _launch_setup(context, *_args, **_kwargs):
        param_overrides = {
            'server.host': LaunchConfiguration('server_host'),
            'server.port': LaunchConfiguration('server_port'),
            'refresh_interval_ms': LaunchConfiguration('refresh_interval_ms'),
        }
        if graph_provider_path:
            param_overrides['plugins'] = ['graph_provider']
            param_overrides['plugins.graph_provider.path'] = graph_provider_path
        param_overrides.update(cors_override(
            LaunchConfiguration('cors_allowed_origins').perform(context),
            LaunchConfiguration('config_file').perform(context), default_config))

        tls_enabled = LaunchConfiguration('tls_enabled').perform(context).lower() in (
            'true', '1', 'yes')
        param_overrides['server.tls.enabled'] = tls_enabled
        cert_file = LaunchConfiguration('cert_file').perform(context)
        key_file = LaunchConfiguration('key_file').perform(context)
        if cert_file:
            param_overrides['server.tls.cert_file'] = cert_file
        if key_file:
            param_overrides['server.tls.key_file'] = key_file
        if tls_enabled and not (cert_file and key_file):
            # The gateway would refuse to start a moment from now, naming the
            # config file. Name the launch arguments instead, here, where they
            # are the thing the reader can actually change.
            print('[gateway.launch.py] TLS is enabled and cert_file/key_file were not both '
                  'given. Pass cert_file:=<path> key_file:=<path>, set them in a config_file, '
                  'or pass tls_enabled:=false to serve plain HTTP. '
                  'scripts/generate_dev_certs.sh makes a self-signed pair for a first run.')

        auth_enabled = LaunchConfiguration('auth_enabled').perform(context).lower() in (
            'true', '1', 'yes')
        param_overrides['auth.enabled'] = auth_enabled
        jwt_secret = LaunchConfiguration('jwt_secret').perform(context)
        clients = LaunchConfiguration('auth_clients').perform(context)
        if jwt_secret:
            param_overrides['auth.jwt_secret'] = jwt_secret
        if clients:
            param_overrides['auth.clients'] = [c for c in clients.split(',') if c]
        if auth_enabled and not jwt_secret:
            # The gateway would refuse to start a moment from now with a
            # message about the config file. Say the actionable thing instead,
            # here, where the launch argument that fixes it is in scope.
            print('[gateway.launch.py] auth is enabled and no jwt_secret was given. '
                  'Pass jwt_secret:=<at least 32 chars> and '
                  'auth_clients:=<id>:<secret>:admin, set them in a config_file, '
                  'or pass auth_enabled:=false to run without authentication.')
        return [Node(
            package='ros2_medkit_gateway',
            executable='gateway_node',
            name='ros2_medkit_gateway',
            output='screen',
            parameters=[default_config, LaunchConfiguration('config_file'), param_overrides],
            arguments=['--ros-args', '--log-level', 'info'])]

    return LaunchDescription([
        declare_override_config_arg,
        declare_host_arg,
        declare_port_arg,
        declare_refresh_arg,
        declare_auth_enabled_arg,
        declare_jwt_secret_arg,
        declare_clients_arg,
        declare_tls_enabled_arg,
        declare_cert_file_arg,
        declare_key_file_arg,
        declare_cors_arg,
        OpaqueFunction(function=_launch_setup),
    ])
