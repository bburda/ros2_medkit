# ros2_medkit_graph_provider

Gateway plugin that builds a live ROS 2 topic dataflow graph per SOVD Function,
enriched with frequency, latency, and drop-rate metrics sourced from `/diagnostics`.
This is the default `IntrospectionProvider` - extracted from `ros2_medkit_gateway`
into a standalone plugin package in v0.4.0.

## What It Does

- Serves `GET /api/v1/functions/{function_id}/x-medkit-graph`, plus a cyclic-subscription
  sampler under the same resource name and a capability href on every Function's
  detail response. That href, and the Function detail's top-level `x-medkit-graph`
  link, exist only while this plugin is loaded - a gateway without it omits both
  rather than publishing a URI nothing answers.
- Exports `describe_plugin_routes`, so the endpoint and the shape of the graph document
  are part of `GET /api/v1/docs` instead of being reachable but undocumented.
- Subscribes to `/diagnostics` and resolves the publishing node for each metrics sample
  (never a fabricated or hardcoded name).
- Builds a per-Function graph of Apps (nodes) and topic connections (edges) with
  per-edge frequency (Hz), latency (ms), and drop-rate (%) metrics. Drop-rate
  degradation only fires if your producer actually emits `drop_rate_percent`/`drop_rate` -
  the reference `greenwave_monitor` producer does not, so that rule is inert against it
  out of the box.
- Reports `pipeline_status` (`healthy` / `degraded` / `broken`) and, when degraded,
  the `bottleneck_edge`.
- Flags each edge's live publisher count (`publisher_count`) and a `rate_ambiguous`
  warning when more than one publisher shares a topic, since the measured rate is then
  summed across publishers and can mask a slow pipeline as healthy.

The metrics require a real `/diagnostics` producer running elsewhere in your system -
see the [Graph Provider tutorial](https://selfpatch.github.io/ros2_medkit/tutorials/graph-provider.html)
for the producer contract before doing anything else. Without one, every edge stays
`pending` forever.

## Configuration

Loaded as a gateway plugin. `gateway.launch.py` auto-detects and loads this plugin
when the package is installed, with no configuration required. To load it manually:

```yaml
plugins: ["graph_provider"]
plugins.graph_provider.path: "/path/to/lib/ros2_medkit_graph_provider/libros2_medkit_graph_provider_plugin.so"
plugins.graph_provider.expected_frequency_hz_default: 30.0
```

See the [configuration reference](https://selfpatch.github.io/ros2_medkit/config/graph-provider.html)
for the full set of thresholds and per-function overrides.

## Architecture

The plugin implements `GatewayPlugin` + `IntrospectionProvider`:

- `GraphProviderPlugin` - Main plugin class with ROS 2 subscriptions
- `GraphBuildState` - Per-request snapshot of topic metrics and app last-seen timestamps
- `GraphBuildConfig` - Resolved thresholds (expected frequency, degradation, freshness)

Entity cache is populated from the ROS 2 graph during each discovery cycle
via the merge pipeline's `PluginLayer`. The graph document itself is always rebuilt
fresh from the live entity cache on every request - there is no cross-request cache
to go stale.

## Documentation

- [Graph Provider Tutorial](https://selfpatch.github.io/ros2_medkit/tutorials/graph-provider.html)
- [Graph Provider Configuration](https://selfpatch.github.io/ros2_medkit/config/graph-provider.html)
- [Plugin System Guide](https://selfpatch.github.io/ros2_medkit/tutorials/plugin-system.html)
- [REST API Reference](https://selfpatch.github.io/ros2_medkit/api/rest.html)

## License

Apache License 2.0
