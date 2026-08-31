#!/usr/bin/env bash
# Reclaim shared-memory segments left behind by DDS participants that were killed
# rather than shut down.
#
# A participant that exits cleanly frees its segments; one that dies on SIGKILL
# does not, and nothing reclaims them afterwards. Measured on Fast DDS: about
# 0.65 MB per participant, and 125 killed participants filled 63 MB of a 64 MB
# /dev/shm. Test runs that kill nodes on purpose - the aggregation suites do -
# accumulate this across every run on a developer machine, and the failure it
# eventually produces looks like anything but a full tmpfs.
#
# The reclaiming is done by `fastdds shm clean`, which is the vendor's own tool
# and ships with every distro we support. It decides a segment is stale by taking
# an exclusive non-blocking flock on the segment's lock file; the kernel drops
# that lock when a process dies, SIGKILL included, so a segment is removed only
# when nothing holds it. Each distro's copy matches its own file naming
# (fastrtps_* on 2.x, fastdds_* on 3.x), so the tool from the sourced
# distribution is the one to call - there is nothing for us to match on.
#
# Refuses to run while ROS processes are alive: the flock test protects live
# segments, but a sweep in the middle of a test run is still a lie about what the
# run measured.
set -euo pipefail

usage() {
  echo "usage: sweep_shm.sh [--force]" >&2
  echo "  --force  sweep even if ROS processes are running" >&2
}

FORCE=0
case "${1:-}" in
  --force) FORCE=1 ;;
  "") ;;
  *) usage; exit 2 ;;
esac

shm_bytes() {
  du -sb /dev/shm 2>/dev/null | cut -f1 || echo 0
}

if [[ "${FORCE}" -eq 0 ]]; then
  # Match on the command line rather than the process name: colcon runs as
  # python3, and a pattern that also matches this script would match itself.
  if pgrep -f '[g]ateway_node|[f]ault_manager_node|[c]test' >/dev/null 2>&1; then
    echo "sweep_shm: ROS processes are running - refusing to sweep (use --force if you mean it)" >&2
    exit 1
  fi
fi

before="$(shm_bytes)"

if command -v fastdds >/dev/null 2>&1; then
  fastdds shm clean || echo "sweep_shm: 'fastdds shm clean' reported a problem, continuing" >&2
else
  echo "sweep_shm: no 'fastdds' on PATH - source a ROS distribution first" >&2
  exit 1
fi

after="$(shm_bytes)"
reclaimed=$((before - after))

printf 'sweep_shm: /dev/shm %s -> %s bytes (reclaimed %s)\n' "${before}" "${after}" "${reclaimed}"
