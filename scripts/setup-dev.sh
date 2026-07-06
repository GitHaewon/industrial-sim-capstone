#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS 2 Jazzy를 찾을 수 없습니다: ${ROS_SETUP}" >&2
  echo "Ubuntu 24.04용 ROS 2 Jazzy를 먼저 설치하세요." >&2
  exit 1
fi

# shellcheck disable=SC1091
set +u
source "${ROS_SETUP}"
set -u
cd "${REPOSITORY_ROOT}"

command -v rosdep >/dev/null || {
  echo "rosdep이 설치되어 있지 않습니다." >&2
  exit 1
}
command -v colcon >/dev/null || {
  echo "colcon이 설치되어 있지 않습니다." >&2
  exit 1
}

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

echo
echo "빌드 완료. 다음 명령으로 작업공간을 활성화하세요:"
echo "  source \"${REPOSITORY_ROOT}/install/setup.bash\""
