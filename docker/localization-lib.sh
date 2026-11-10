#!/bin/bash
#
# localization-lib.sh - Launch-file and bag picker shared by both images'
# menus (entrypoint-standalone.sh and entrypoint-sim.sh). Defines functions
# only; nothing runs at the top level. Expects log_warn and the C_* color
# variables to be defined by whoever sources it.

BAGS_DIR="$HOME/ros2_ws/install/uwb_localization/share/uwb_localization/bags"

# choose_localization_launch: asks which launch file and which bag to use.
# Sets LAUNCH_FILE and LAUNCH_ARGS (an array of `name:=value` arguments).
choose_localization_launch() {
  echo
  echo "Which localization launch file do you want to run?"
  echo "  1) localization.launch.py         -- UWB + odometry (params_sim.yaml + RViz)"
  echo "  2) localization_dataset.launch.py -- UWB + odometry + radar (params_dataset.yaml)"
  local launch_choice
  read -r -p "Select [1-2] (default 1): " launch_choice
  LAUNCH_FILE="localization.launch.py"
  [[ "$launch_choice" == "2" ]] && LAUNCH_FILE="localization_dataset.launch.py"

  # Both launch files take a `bag_name` launch argument (relative to
  # uwb_localization/bags/, bind-mounted from the host -- see
  # docker-compose.yml) instead of a hardcoded path, so offer whatever bags
  # are actually sitting there. Recognizes directory-style bags (a
  # metadata.yaml one level down) and standalone .mcap bags.
  local -a bag_choices=()
  if [[ -d "$BAGS_DIR" ]]; then
    while IFS= read -r -d '' metadata; do
      bag_choices+=("$(dirname "${metadata#"$BAGS_DIR"/}")")
    done < <(find "$BAGS_DIR" -mindepth 2 -maxdepth 2 -name metadata.yaml -print0 2>/dev/null)
    while IFS= read -r -d '' mcap; do
      bag_choices+=("${mcap#"$BAGS_DIR"/}")
    done < <(find "$BAGS_DIR" -mindepth 1 -name '*.mcap' -print0 2>/dev/null)
  fi

  local bag_name=""
  if [[ ${#bag_choices[@]} -gt 0 ]]; then
    echo
    echo "Available bags (drop more into uwb_localization/bags/ on the host):"
    local i=1 b
    for b in "${bag_choices[@]}"; do
      echo "  $i) $b"
      i=$((i + 1))
    done
    echo "  0) Use the launch file's built-in default"
    local bag_choice
    read -r -p "Select a bag [0-${#bag_choices[@]}] (default 0): " bag_choice
    if [[ "$bag_choice" =~ ^[0-9]+$ ]] && (( bag_choice >= 1 && bag_choice <= ${#bag_choices[@]} )); then
      bag_name="${bag_choices[$((bag_choice - 1))]}"
    fi
  else
    log_warn "No bags found in uwb_localization/bags/ on the host -- using the launch file's built-in default."
    log_warn "Drop a recorded rosbag there (see that folder's README.md) to select it here next time."
  fi

  LAUNCH_ARGS=()
  [[ -n "$bag_name" ]] && LAUNCH_ARGS+=("bag_name:=$bag_name")
  return 0
}
