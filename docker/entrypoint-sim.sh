#!/bin/bash
#
# entrypoint-sim.sh - Interactive entrypoint for the mr-radio-localization
# SITL image (Dockerfile.sim; installed in the image as /entrypoint.sh).
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
# shellcheck disable=SC1091
source /localization-lib.sh

REPO_DIR="$HOME/ros2_ws/src/mr-radio-localization"

print_welcome() {
  cat <<EOF

${C_CYAN}${C_BOLD}==================================================
  mr-radio-localization -- SITL dev container
==================================================${C_RESET}

PX4 SITL, Gazebo Harmonic, and the ROS 2 UWB relative-localization
stack, pre-built and ready to go.

${C_DIM}Tip: your whole UWBPX4Sim checkout (UWBPX4SIM_DIR on the host) is
bind-mounted -- edit a layout, a plugin-parameter preset, or drop in a
custom .sdf world there, then re-run setup (option 1) to apply it.
uwb_localization/config and uwb_localization/bags are bind-mounted too.

Tip: for Gazebo/RViz windows to appear, run on the HOST
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
  # Same check as UWBPX4Sim's own print_menu: .setup_env lives in the
  # host-mounted checkout and outlives the container, while the built
  # plugin lives in this container's PX4 tree -- both must exist.
  local setup_tick=""
  local plugin_lib="$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins/libUWBGazeboPlugin.so"
  if [[ -f "$UWBPX4SIM_DIR/.setup_env" && -f "$plugin_lib" ]]; then
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
  choose_localization_launch
  local launch_file="$LAUNCH_FILE"
  local -a launch_args=("${LAUNCH_ARGS[@]}")

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
