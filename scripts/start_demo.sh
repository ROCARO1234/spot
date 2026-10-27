#!/usr/bin/env bash
# No arguments: offline check only. No hardware is enabled by this wrapper.
set -euo pipefail
release_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
python_bin=${ROBOT_PYTHON:-"$HOME/Desktop/Spot_SM5_RoboFest_v1/spot/.venv/bin/python"}
if [[ ! -x "$python_bin" ]]; then
  if [[ -n ${ROBOT_PYTHON:-} ]]; then
    printf 'ROBOT_PYTHON is not executable: %s\n' "$python_bin" >&2
    exit 2
  fi
  python_bin=$(command -v python3)
fi
config_dir=${SPOT_CONFIG_DIR:-"$HOME/Desktop/sm5_gait_fixed/config"}
hardware=false
for argument in "$@"; do
  [[ "$argument" != --hardware ]] || hardware=true
done
if "$hardware"; then
  if [[ ! -t 0 ]]; then
    printf 'Hardware requires an interactive terminal and operator confirmation.\n' >&2
    exit 2
  fi
  printf 'Confirm robot supported, aligned to the requested initial pose, area clear and physical disconnect ready.\n'
  read -r -p 'Type ROBOT SUPPORTED to permit this hardware run: ' confirmation
  if [[ "$confirmation" != 'ROBOT SUPPORTED' ]]; then
    printf 'Cancelled; controller not started.\n' >&2
    exit 2
  fi
fi
if [[ $# -eq 0 ]]; then
  set -- --check
fi
exec "$python_bin" "$release_dir/runner_gait_ikfin.py" \
  --config "$config_dir/robot.json" --poses "$config_dir/poses.json" "$@"
