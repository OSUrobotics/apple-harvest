#!/usr/bin/env bash
set -euo pipefail

UI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${UI_DIR}/.." && pwd)"
# VENV_DIR="${HARVEST_VENV:-/home/jn2/college/Fin_Ray_Gripper/.venv}"
APPLE_GRIPPER_WS="${APPLE_GRIPPER_WS:-/home/jn2/college/Forked_Repos/apple_gripper}"
FINRAY_WS="${FINRAY_WS:-/home/jn2/college/Fin_Ray_Gripper}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
APPLE_WS="${APPLE_WS:-/home/imml/apple_harvest_ws}"

# source "${VENV_DIR}/bin/activate"
# ROS/ament setup scripts legitimately inspect variables that may not exist
# yet, so nounset must be disabled while the overlays are sourced.
set +u
source "${ROS_SETUP}"
# source "${REPO_DIR}/install/local_setup.bash"
# source "${APPLE_GRIPPER_WS}/install/local_setup.bash"
# source "${FINRAY_WS}/install/local_setup.bash"
source "${APPLE_WS}/install/local_setup.bash"
set -u
# export PYTHONPATH="${VENV_DIR}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

cd "${UI_DIR}"
exec python3 -m harvest_ui "$@"
