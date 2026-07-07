#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SKIP_BUILD=0

if [[ "${1:-}" == "--skip-build" ]]; then
  SKIP_BUILD=1
  shift
fi

cd "${REPO_ROOT}"
source /opt/ros/jazzy/setup.bash

if [[ "${SKIP_BUILD}" == "0" ]]; then
  colcon build --symlink-install \
    --packages-select \
    amr_control \
    factory_description \
    conveyor_control \
    arm_control \
    item_vision \
    factory_manager
fi

source install/setup.bash

exec ros2 launch factory_description demo.launch.py "$@"
