#!/usr/bin/env bash
set -euo pipefail

UI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Original computer (Humble, separate gripper workspaces):
#REPO_DIR="$(cd -- "${UI_DIR}/.." && pwd)"
#VENV_DIR="${HARVEST_VENV:-/home/jn2/college/Fin_Ray_Gripper/.venv}"
#APPLE_GRIPPER_WS="${APPLE_GRIPPER_WS:-/home/jn2/college/Forked_Repos/apple_gripper}"
#FINRAY_WS="${FINRAY_WS:-/home/jn2/college/Fin_Ray_Gripper}"
#ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
#APPLE_WS="${APPLE_WS:-/home/imml/apple_harvest_ws}"

# JN2-alt (Jazzy, single apple_harvest_ws with .venv at its root):
APPLE_WS="${APPLE_WS:-$(cd -- "${UI_DIR}/../../.." && pwd)}"
VENV_DIR="${HARVEST_VENV:-${APPLE_WS}/.venv}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
# Nodes talk through the zenoh router (ros2 run rmw_zenoh_cpp rmw_zenohd).
#export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "Virtual environment not found at ${VENV_DIR}; see README.md." >&2
  exit 1
fi

# ROS/ament setup scripts legitimately inspect variables that may not exist
# yet, so nounset must be disabled while the overlays are sourced.
set +u
# Original computer:
#source "${VENV_DIR}/bin/activate"
#source "${ROS_SETUP}"
#source "${REPO_DIR}/install/local_setup.bash"
#source "${APPLE_GRIPPER_WS}/install/local_setup.bash"
#source "${FINRAY_WS}/install/local_setup.bash"
#source "${APPLE_WS}/install/local_setup.bash"

# This computer:
source "${ROS_SETUP}"
source "${APPLE_WS}/install/local_setup.bash"
source "${VENV_DIR}/bin/activate"
set -u

# Original computer:
#export PYTHONPATH="${VENV_DIR}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

# This computer: ROS node executables use #!/usr/bin/python3, so child
# processes only see the venv packages through PYTHONPATH.
VENV_SITE="$("${VENV_DIR}/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export PYTHONPATH="${VENV_SITE}${PYTHONPATH:+:${PYTHONPATH}}"

cd "${UI_DIR}"
# Original computer:
#exec python3 -m harvest_ui "$@"
exec python -m harvest_ui "$@"
