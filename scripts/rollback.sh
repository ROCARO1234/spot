#!/usr/bin/env bash
# Run the retained original installation. No files are deleted or replaced.
set -euo pipefail
original=${SPOT_ORIGINAL_DIR:-"$HOME/Desktop/Spot_SM5_RoboFest_v1/spot"}
if [[ ! -f "$original/runner_gait_ikfin.py" ]]; then
  printf 'Original standalone controller not found: %s\n' "$original" >&2
  exit 2
fi
python_bin=${ROBOT_PYTHON:-"$original/.venv/bin/python"}
if [[ ! -x "$python_bin" ]]; then
  if [[ -n ${ROBOT_PYTHON:-} ]]; then
    printf 'ROBOT_PYTHON is not executable: %s\n' "$python_bin" >&2
    exit 2
  fi
  python_bin=$(command -v python3)
fi
hardware=false
for argument in "$@"; do
  [[ "$argument" != --hardware ]] || hardware=true
done
if "$hardware"; then
  if [[ ! -t 0 ]]; then
    printf 'Hardware rollback requires an interactive terminal.\n' >&2
    exit 2
  fi
  printf 'Confirm the robot is supported and aligned to STAND, with physical disconnect ready.\n'
  read -r -p 'Type ROBOT SUPPORTED to permit this hardware run: ' confirmation
  [[ "$confirmation" == 'ROBOT SUPPORTED' ]] || exit 2
fi
if [[ $# -eq 0 ]]; then
  set -- --check
fi
config_dir=${SPOT_CONFIG_DIR:-"$HOME/Desktop/sm5_gait_fixed/config"}
printf 'Using retained original installation: %s\n' "$original"
exec "$python_bin" "$original/runner_gait_ikfin.py" \
  --config "$config_dir/robot.json" --poses "$config_dir/poses.json" "$@"
