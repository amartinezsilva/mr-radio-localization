#!/bin/bash
#
# entrypoint.sh - Interactive entrypoint for the mr-radio-localization image.
#
# Extends UWBPX4Sim's own entrypoint (sourced below from
# /uwbpx4sim-entrypoint-lib.sh, baked into the base image) rather than
# duplicating it: that library defines print_welcome/print_menu/run_setup/
# run_demo/do_exit/uwbpx4sim_main and friends as plain functions with no
# top-level execution. Redefining print_menu/run_menu here after sourcing
# it, then calling the same uwbpx4sim_main, works because bash resolves a
# function call by name at the time it actually runs -- uwbpx4sim_main's
# internal `run_menu` call below will invoke *this* file's run_menu, not
# the base library's, since this one was defined more recently in the
# same shell.
#
# See UWBPX4Sim/docker/entrypoint.sh's header for the PID 1 / persistent
# container / "Exit" mechanics -- all inherited unchanged from the base
# image via the sourced library.

set -e

# shellcheck disable=SC1091
source /uwbpx4sim-entrypoint-lib.sh

REPO_DIR="$HOME/ros2_ws/src/mr-radio-localization"

print_welcome() {
  cat <<EOF

${C_CYAN}${C_BOLD}==================================================
  mr-radio-localization -- SITL dev container
==================================================${C_RESET}

PX4 SITL, Gazebo Harmonic, and the ROS 2 UWB relative-localization
stack, pre-built and ready to go.

${C_DIM}Tip: UWBPX4Sim/config, uwb_gazebo_plugin, and worlds are bind-mounted
from the host -- edit a layout, params.yaml, or drop in a custom .sdf
world there, then re-run setup (option 1) to apply it.

Tip: for QGroundControl/Gazebo windows to appear, run on the HOST
first: xhost +local:docker (see docker-compose.yml's comments for the
extra step Wayland/XWayland desktops need).

Tip: this container keeps running in the background even if you close
this terminal -- run setup.sh again (or docker exec -it <container>
/entrypoint.sh) any time to get back to this menu with everything you
configured still in place. Pick "Exit" below when you actually want to
stop it.${C_RESET}
EOF
}

print_menu() {
  # Tick shown next to "Launch simulation" once setup_simulator.sh has run
  # and written its chosen layout/world/PX4_DIR to .setup_env -- lets the
  # user tell at a glance, before picking option 2, whether a setup is
  # already loaded or still needs to be run first. Checked fresh on every
  # call (not cached) so it reflects setup having *just* finished too.
  local setup_tick=""
  if [[ -f "$UWBPX4SIM_DIR/.setup_env" ]]; then
    setup_tick=" ${C_GREEN}✓ configured${C_RESET}"
  fi
  cat <<EOF

${C_CYAN}==================================================${C_RESET}
 ${C_BOLD}mr-radio-localization${C_RESET} dev container
${C_CYAN}==================================================${C_RESET}
  ${C_GREEN}1)${C_RESET} Run UWBPX4Sim setup (setup_simulator.sh)
     Configure the PX4/Gazebo plugin, models, and layout.
  ${C_GREEN}2)${C_RESET} Launch simulation${setup_tick}
     Launch PX4 SITL + Gazebo + the UWB bridge/offboard nodes.
  ${C_GREEN}3)${C_RESET} Launch localization
     Run the relative-localization estimator against the running simulation.
  ${C_GREEN}4)${C_RESET} Open a shell
  ${C_YELLOW}5)${C_RESET} Exit
${C_CYAN}==================================================${C_RESET}
EOF
}

run_localization() {
  local bags_dir="$HOME/ros2_ws/install/uwb_localization/share/uwb_localization/bags"

  echo
  echo "Which localization launch file do you want to run?"
  echo "  1) localization.launch.py         -- needs the simulation running (params_sim.yaml + RViz)"
  echo "  2) localization_dataset.launch.py -- self-contained offline replay (params_dataset.yaml + radar odometry)"
  local launch_choice
  read -r -p "Select [1-2] (default 1): " launch_choice
  local launch_file="localization.launch.py"
  [[ "$launch_choice" == "2" ]] && launch_file="localization_dataset.launch.py"

  # Both launch files take a `bag_name` launch argument (relative to
  # uwb_localization/bags/, bind-mounted from the host -- see
  # docker-compose.yml) instead of a hardcoded path, so offer whatever bags
  # are actually sitting there rather than making the user hand-edit the
  # launch file every time they want to try a different one. Recognizes
  # classic directory-style bags (a metadata.yaml one level down) and
  # standalone .mcap bags (like the bundled experiment1_gt_v2_0 one).
  local -a bag_choices=()
  if [[ -d "$bags_dir" ]]; then
    while IFS= read -r -d '' metadata; do
      bag_choices+=("$(dirname "${metadata#"$bags_dir"/}")")
    done < <(find "$bags_dir" -mindepth 2 -maxdepth 2 -name metadata.yaml -print0 2>/dev/null)
    while IFS= read -r -d '' mcap; do
      bag_choices+=("${mcap#"$bags_dir"/}")
    done < <(find "$bags_dir" -mindepth 1 -name '*.mcap' -print0 2>/dev/null)
  fi

  local bag_name=""
  if [[ ${#bag_choices[@]} -gt 0 ]]; then
    echo
    echo "Available bags (drop more into uwb_localization/bags/ on the host):"
    local i=1
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

  local -a launch_args=()
  [[ -n "$bag_name" ]] && launch_args+=("bag_name:=$bag_name")

  # simulator_launcher.sh's tmux session name (see that script's SESSION
  # var) -- localization.launch.py needs the simulation actually running,
  # so inject a new window there, automating the "Ctrl-b c, then run ros2
  # launch ..." step this project's README used to document as manual.
  # localization_dataset.launch.py is self-contained (plays its own bag,
  # no simulation needed), so it gets its own session when the sim one
  # isn't up instead of being blocked on it.
  local session="${SESSION:-sim_session}"
  local target_session="$session"

  if [[ "$launch_file" == "localization.launch.py" ]]; then
    if ! tmux has-session -t "$session" 2>/dev/null; then
      echo
      log_warn "Simulation doesn't look like it's running (no '$session' tmux session)."
      if confirm_yes "Launch the simulation now instead?"; then
        run_demo
      fi
      return 1
    fi
  elif ! tmux has-session -t "$session" 2>/dev/null; then
    target_session="localization_session"
  fi

  echo
  local launch_cmd="bash -lc 'source \"$HOME/ros2_ws/install/setup.bash\" && ros2 launch uwb_localization $launch_file ${launch_args[*]}; exec bash'"
  if tmux has-session -t "$target_session" 2>/dev/null; then
    log_info "Launching '$launch_file' in a new tmux window in '$target_session'."
    tmux new-window -t "$target_session" -n localization "$launch_cmd"
  else
    log_info "Launching '$launch_file' in a new tmux session '$target_session'."
    tmux new-session -d -s "$target_session" -n localization "$launch_cmd"
  fi
  log_info "Attach with: tmux attach -t $target_session"
  log_ok "Localization launched. Back to the menu."
}

run_menu() {
  local choice
  print_welcome
  while true; do
    print_menu
    read -r -p "Select an option [1-5]: " choice || { echo; exec bash; }
    case "$choice" in
      1) run_setup || true ;;
      2) run_demo || true ;;
      3) run_localization || true ;;
      4) bash || true; log_info "Shell closed. Back to the menu." ;;
      5) do_exit ;;
      *) echo "Invalid selection: '$choice'" ;;
    esac
  done
}

uwbpx4sim_main "$@"
