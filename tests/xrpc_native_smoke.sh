#!/usr/bin/env bash
set -euo pipefail
runtime=$(mktemp -d "${TMPDIR:-/tmp}/runtime-sync-xrpc.XXXXXX")
cleanup() {
  python3 - "$runtime" <<'PY'
import shutil, sys
shutil.rmtree(sys.argv[1])
PY
}
trap cleanup EXIT
export ROS_HOME="$runtime/ros-home"
export ROS_LOG_DIR="$runtime/ros-logs"
# rostest starts its own master; no inherited live ROS graph is consumed.
unset ROS_MASTER_URI
rostest periodic_sync runtime_node_smoke.test control_socket:="$runtime/control.sock"
