#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-runtime-sync >/dev/null
test "$(rospack find periodic_sync)" = "/opt/ros/${ROS_DISTRO}/share/periodic_sync"
test -x "/opt/ros/${ROS_DISTRO}/lib/periodic_sync/sync_coordinator"
for message in SyncReady SyncTrigger SyncAck SyncParticipantStats SyncStatistics SyncEvent; do
  test -f "/opt/ros/${ROS_DISTRO}/include/periodic_sync/${message}.h"
  test -f "/opt/ros/${ROS_DISTRO}/share/periodic_sync/msg/${message}.msg"
done

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/periodic_sync" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
