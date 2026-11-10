#!/bin/bash
#
# entrypoint-standalone.sh - Interactive entrypoint for the standalone
# mr-radio-localization image (Dockerfile; installed in the image as
# /entrypoint.sh). No simulator here: it runs the localization stack
# against rosbags dropped into uwb_localization/bags/ on the host.
#
# Same container lifecycle as the SITL image (and UWBPX4Sim's own image),
# so setup.sh drives both the same way: setup.sh keeps the container alive
# with an idle PID 1 and `docker exec`s this script for the menu. "Exit"
# signals PID 1 to stop the container, and exits with REMOVE_EXIT_CODE when
# the user asks for the container to be removed, which setup.sh acts on.

set -e

if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
  C_RESET=$'\033[0m'
  C_BOLD=$'\033[1m'
  C_DIM=$'\033[2m'
  C_CYAN=$'\033[36m'
  C_GREEN=$'\033[32m'
  C_YELLOW=$'\033[33m'
else
  C_RESET="" C_BOLD="" C_DIM="" C_CYAN="" C_GREEN="" C_YELLOW=""
fi

log_info() { printf '%s[INFO] %s%s\n' "$C_CYAN" "$1" "$C_RESET"; }
log_warn() { printf '%s[WARN] %s%s\n' "$C_YELLOW" "$1" "$C_RESET" >&2; }
log_ok()   { printf '%s[OK] %s%s\n' "$C_GREEN" "$1" "$C_RESET"; }

confirm_yes() {
  local reply
  read -r -p "$1 [y/N] " reply || reply=""
  [[ "$reply" =~ ^[Yy]$ ]]
}

# shellcheck disable=SC1091
source /localization-lib.sh

# Must match setup.sh's check after `docker exec ... /entrypoint.sh`.
REMOVE_EXIT_CODE=99

print_welcome() {
  cat <<EOF

${C_CYAN}${C_BOLD}==================================================
  mr-radio-localization -- standalone container
==================================================${C_RESET}

The ROS 2 UWB relative-localization stack, pre-built and ready to run
against your own rosbags. No simulator in this image -- for the PX4/Gazebo
SITL simulation, use the SITL image instead (docker/setup.sh --sim).

${C_DIM}Tip: uwb_localization/bags and uwb_localization/config are bind-mounted
from the host -- drop your bags into bags/ and tune params_sim.yaml /
params_dataset.yaml in config/, no rebuild needed.

Tip: for RViz to appear, run on the HOST first: xhost +local:docker
(setup.sh does this for you).

Tip: this container keeps running in the background even if you close
this terminal -- run setup.sh again (or docker exec -it <container>
/entrypoint.sh) any time to get back to this menu. Pick "Exit" below when
you actually want to stop it.${C_RESET}
EOF
}

print_menu() {
  cat <<EOF

${C_CYAN}==================================================${C_RESET}
 ${C_BOLD}mr-radio-localization${C_RESET} standalone container
${C_CYAN}==================================================${C_RESET}
  ${C_GREEN}1)${C_RESET} Launch localization
     Run the relative-localization estimator on a bag from uwb_localization/bags/.
  ${C_GREEN}2)${C_RESET} Open a shell
  ${C_YELLOW}3)${C_RESET} Exit
${C_CYAN}==================================================${C_RESET}
EOF
}

run_localization() {
  choose_localization_launch
  echo
  log_info "Launching $LAUNCH_FILE ${LAUNCH_ARGS[*]} -- press Ctrl-C to stop it and return to the menu."
  # Runs in the foreground. The INT trap keeps Ctrl-C from killing this
  # menu along with the launch: a trapped (not ignored) signal is reset to
  # its default in child processes, so ros2 launch still receives it.
  trap ':' INT
  local rc=0
  ( source /opt/ros/jazzy/setup.bash && \
    source "$HOME/ros2_ws/install/setup.bash" && \
    ros2 launch uwb_localization "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}" ) || rc=$?
  trap - INT
  log_info "Localization stopped (exit code $rc). Back to the menu."
}

do_exit() {
  log_info "Exiting."
  if [[ $$ -eq 1 ]]; then
    # We ARE the container's main process (plain `docker run`/`docker
    # compose run --rm`) -- exiting already stops the container.
    exit 0
  fi
  # A `docker exec`'d session: PID 1 is setup.sh's idle placeholder, so
  # signal it to stop the whole container (we share its PID namespace).
  # Exit right away instead of waiting for PID 1 to die -- Docker's
  # teardown would otherwise SIGKILL this session before it can return
  # REMOVE_EXIT_CODE to setup.sh.
  if confirm_yes "Remove the container completely instead of just stopping it?"; then
    log_info "Stopping the background container for removal."
    kill -TERM 1 2>/dev/null || true
    exit "$REMOVE_EXIT_CODE"
  fi
  log_info "Stopping the background container."
  kill -TERM 1 2>/dev/null || true
  exit 0
}

run_menu() {
  local choice
  print_welcome
  while true; do
    print_menu
    read -r -p "Select an option [1-3]: " choice || { echo; exec bash; }
    case "$choice" in
      1) run_localization || true ;;
      2) bash || true; log_info "Shell closed. Back to the menu." ;;
      3) do_exit ;;
      *) echo "Invalid selection: '$choice'" ;;
    esac
  done
}

if [[ $# -gt 0 && "$1" != "menu" ]]; then
  exec "$@"
fi

if [[ ! -t 0 ]]; then
  log_info "No interactive terminal attached; starting a shell instead of the menu."
  log_info "Re-run with an interactive TTY (./setup.sh, or docker exec -it <container> /entrypoint.sh) for the menu."
  exec bash
fi

run_menu
