#!/bin/bash
#
# entrypoint.sh - Interactive entrypoint for the mr-radio-localization image.
#
# With no arguments (or "menu"), prompts the user to either run UWBPX4Sim's
# guided setup or launch the SITL demo. Any other arguments are exec'd
# directly, so `docker run <image> <cmd>` still works as a plain override.

set -e

log_info() { printf '[INFO] %s\n' "$1"; }
log_warn() { printf '[WARN] %s\n' "$1" >&2; }

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

REPO_DIR="$HOME/ros2_ws/src/mr-radio-localization"
UWBPX4SIM_DIR="$REPO_DIR/UWBPX4Sim"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

print_menu() {
  cat <<'EOF'

==================================================
 mr-radio-localization dev container
==================================================
  1) Run UWBPX4Sim setup (setup_simulator.sh)
     Configure the PX4/Gazebo plugin, models, and layout.
  2) Run the demo simulation
     Launch PX4 SITL + Gazebo + the UWB bridge/offboard nodes.
  3) Open a shell
==================================================
EOF
}

run_setup() {
  if [[ ! -f "$UWBPX4SIM_DIR/setup_simulator.sh" ]]; then
    log_warn "setup_simulator.sh not found in $UWBPX4SIM_DIR."
    log_warn "The image's UWBPX4Sim checkout may predate that script, or your bind"
    log_warn "mounts may be pointed at the wrong path. Rebuild the image, or check"
    log_warn "docker-compose.yml's volumes, then try again."
    return 1
  fi
  cd "$UWBPX4SIM_DIR"
  exec ./setup_simulator.sh
}

run_demo() {
  if [[ ! -d "$PX4_DIR/src/modules/simulation/gz_plugins/uwb_gazebo_plugin" ]]; then
    echo
    log_warn "UWBPX4Sim does not look set up yet under $PX4_DIR."
    if confirm_yes "Run the setup now instead?"; then
      run_setup
    fi
    return 1
  fi

  if [[ ! -f "$UWBPX4SIM_DIR/simulator_launcher.sh" ]]; then
    log_warn "simulator_launcher.sh not found in $UWBPX4SIM_DIR."
    return 1
  fi

  echo
  log_info "Launching the SITL demo. Once tmux attaches, open a new window"
  log_info "(Ctrl-b c) and run:"
  log_info "  ros2 launch uwb_localization localization.launch.py"
  log_info "to see the relative localization estimate."
  echo
  read -r -p "Press Enter to continue (Ctrl-C to cancel)... " _ || true
  cd "$UWBPX4SIM_DIR"
  exec ./simulator_launcher.sh
}

confirm_yes() {
  local reply
  read -r -p "$1 [y/N] " reply || reply=""
  [[ "$reply" =~ ^[Yy]$ ]]
}

run_menu() {
  local choice
  while true; do
    print_menu
    read -r -p "Select an option [1-3]: " choice || { echo; exec bash; }
    case "$choice" in
      1) run_setup || true ;;
      2) run_demo || true ;;
      3) exec bash ;;
      *) echo "Invalid selection: '$choice'" ;;
    esac
  done
}

if [[ $# -gt 0 && "$1" != "menu" ]]; then
  exec "$@"
fi

if [[ ! -t 0 ]]; then
  log_info "No interactive terminal attached; starting a shell instead of the menu."
  log_info "Re-run with an interactive TTY (docker compose run --rm app) for the menu."
  exec bash
fi

run_menu
