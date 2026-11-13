#!/bin/bash
#
# localization-lib.sh - Launch-file and bag picker, and the rebuild of
# uwb_localization from the mounted checkout, shared by both images' menus
# (entrypoint-standalone.sh and entrypoint-sim.sh). Defines functions only;
# nothing runs at the top level. Expects log_info/log_warn/log_ok and the
# C_* color variables to be defined by whoever sources it.

# The host checkout, mounted over the image's own clone (docker-compose.yml).
LOCALIZATION_SRC="$HOME/ros2_ws/src/mr-radio-localization"
# The installed bags/ is a link to this folder, so listing it here is the same.
BAGS_DIR="$LOCALIZATION_SRC/uwb_localization/bags"
# Source state the current uwb_localization binaries were built from: written
# by the Dockerfile for the image's build, and by rebuild_localization.
BUILD_STATE_FILE="$HOME/.uwb_localization_build_state"

# localization_source_state: a hash of the files that are compiled -- the
# contents of uwb_localization's CMakeLists.txt, package.xml, src/,
# include/ and msg/ (new, untracked files included). Commits, docs,
# parameters and launch files don't count: the installed config/ and
# launch/ are links into the checkout, so edits there are live without a
# rebuild. Also called by the Dockerfiles for the image's own build.
localization_source_state() {
  local pkg="$LOCALIZATION_SRC/uwb_localization"
  [[ -f "$pkg/CMakeLists.txt" ]] || return 1
  ( cd "$pkg" && find CMakeLists.txt package.xml src include msg -type f -print0 2>/dev/null \
      | sort -z | xargs -0 sha1sum | sha1sum | cut -d' ' -f1 )
}

# localization_needs_rebuild: true when the compiled files of the mounted
# checkout differ from the ones the binaries were built from. False when it
# can't tell (no checkout, no recorded state).
localization_needs_rebuild() {
  local current built
  current=$(localization_source_state) || return 1
  built=$(cat "$BUILD_STATE_FILE" 2>/dev/null) || return 1
  [[ "$current" != "$built" ]]
}

_colcon_build_localization() {
  ( source /opt/ros/jazzy/setup.bash && cd "$HOME/ros2_ws" && \
    colcon build --symlink-install --packages-select uwb_localization --cmake-args -DBUILD_TESTING=OFF )
}

# rebuild_localization: rebuilds uwb_localization from the mounted checkout
# (incremental) and records the source state it was built from. If the
# incremental build fails, offers a clean one: an incremental build can't
# recover from some changes, e.g. a removed .msg/.srv file leaves stale
# generated code behind in build/.
rebuild_localization() {
  local state
  state=$(localization_source_state) || state=""
  log_info "Rebuilding uwb_localization from your checkout (incremental; the first time takes a couple of minutes)..."
  if ! _colcon_build_localization; then
    echo
    log_warn "The incremental build failed (errors above)."
    if ! confirm_yes "Retry with a clean build of uwb_localization (deletes its build/ folder only)?"; then
      log_warn "The previous binaries are still in place."
      return 1
    fi
    # Only build/: install/uwb_localization holds the bags/ bind mount from
    # the host, so it must never be deleted recursively.
    rm -rf "$HOME/ros2_ws/build/uwb_localization"
    if ! _colcon_build_localization; then
      log_warn "The clean build failed too -- see the errors above."
      return 1
    fi
  fi
  [[ -n "$state" ]] && echo "$state" > "$BUILD_STATE_FILE"
  log_ok "uwb_localization rebuilt."
}

# offer_rebuild_if_needed: before launching, asks whether to rebuild first
# when the binaries are older than the mounted source.
offer_rebuild_if_needed() {
  localization_needs_rebuild || return 0
  echo
  log_warn "Your mr-radio-localization checkout differs from the source uwb_localization was last built from."
  if confirm_yes "Rebuild it first?"; then
    rebuild_localization || true
  fi
}

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
  # uwb_localization/bags/, in the checkout mounted from the host -- see
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
