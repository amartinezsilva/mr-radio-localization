#!/bin/bash
#
# setup.sh - Build (and optionally launch) the mr-radio-localization image.
#
# Wraps `docker compose build` so you don't need to remember the compose
# invocation or run it from this exact directory. Checks Docker/Compose are
# available, tells you about any existing image (and lets you just launch
# it instead of rebuilding), warns about low disk space and offers to
# reclaim some first (the image alone is ~25GB, and a build has failed
# mid-way from running out of disk before), builds, then -- unless told not
# to -- also runs the X11 setup and `docker compose run` for you, so
# nothing has to be copy-pasted.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors only when writing to a real terminal, and never if the caller
# opted out via the NO_COLOR convention (https://no-color.org) -- matches
# entrypoint.sh's convention so the two scripts feel consistent.
if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
  USE_COLOR=1
  C_RESET=$'\033[0m'
  C_BOLD=$'\033[1m'
  C_DIM=$'\033[2m'
  C_CYAN=$'\033[36m'
  C_GREEN=$'\033[32m'
  C_YELLOW=$'\033[33m'
  C_RED=$'\033[31m'
else
  USE_COLOR=""
  C_RESET="" C_BOLD="" C_DIM="" C_CYAN="" C_GREEN="" C_YELLOW="" C_RED=""
fi

ASSUME_YES=0
DRY_RUN=0
NO_CACHE=0
RUN_AFTER=""      # "", "yes", "no"
FORCE_ACTION=""   # "", "build", "launch"
IMAGE="mr-radio-localization:jazzy"
MIN_FREE_GB=35   # a build has failed mid-export with as much as 24-28GB free in practice;
                 # successful builds in the same environment needed ~40GB+ of real headroom

# Log helpers keep the literal "[INFO]"/"[WARN]"/etc. tags uncolored and
# contiguous (color wraps the whole line from outside), so anything that
# greps for those tags -- a human skimming, or a test harness -- still
# matches regardless of whether color is on.
log_step()  { printf '\n%s==> %s%s\n' "${C_BOLD}${C_CYAN}" "$1" "$C_RESET"; }
log_info()  { printf '%s[INFO] %s%s\n' "$C_CYAN" "$1" "$C_RESET"; }
log_warn()  { printf '%s[WARN] %s%s\n' "$C_YELLOW" "$1" "$C_RESET" >&2; }
log_error() { printf '%s[ERROR] %s%s\n' "$C_RED" "$1" "$C_RESET" >&2; }
log_ok()    { printf '%s[OK] %s%s\n' "$C_GREEN" "$1" "$C_RESET"; }

confirm() {
  # confirm "question" [default: y|n]
  local prompt="$1" default="${2:-n}" reply
  if (( ASSUME_YES )); then
    [[ "$default" == "y" ]]
    return
  fi
  if [[ "$default" == "y" ]]; then
    read -r -p "$prompt [Y/n] " reply || reply=""
    [[ -z "$reply" || "$reply" =~ ^[Yy]$ ]]
  else
    read -r -p "$prompt [y/N] " reply || reply=""
    [[ "$reply" =~ ^[Yy]$ ]]
  fi
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    log_error "Missing required command: $1"
    exit 1
  fi
}

print_usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Builds (or launches) the mr-radio-localization Docker image ($IMAGE).

Options:
  --rebuild       Skip the existing-image prompt and always rebuild
  --launch, -l    Skip the existing-image prompt and always launch the
                   existing image directly, without building (errors if
                   no image exists yet)
  --no-cache      Force a full rebuild, ignoring cached layers
  --run           After building/launching, run the X11 setup and start
                   the container (xhost + docker compose run --rm app),
                   without asking
  --no-run        Skip that and just print the commands, without asking
  -y, --yes       Non-interactive: proceed with recommended defaults
                   (rebuild if an image exists, run afterward), unless
                   --launch or --no-run are also given
  -n, --dry-run   Print what would happen without changing anything
  -h, --help      Show this help message

A full build compiles PX4, ROS 2 packages, and more from scratch --
expect roughly 15-30 minutes, and real disk headroom (~40GB+) the
first time or after --no-cache invalidates the early layers.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rebuild) FORCE_ACTION="build"; shift ;;
    --launch|-l) FORCE_ACTION="launch"; shift ;;
    --no-cache) NO_CACHE=1; shift ;;
    --run) RUN_AFTER="yes"; shift ;;
    --no-run) RUN_AFTER="no"; shift ;;
    -y|--yes) ASSUME_YES=1; shift ;;
    -n|--dry-run) DRY_RUN=1; shift ;;
    -h|--help) print_usage; exit 0 ;;
    *) log_error "Unknown argument: $1"; print_usage >&2; exit 1 ;;
  esac
done

cat <<EOF
${C_CYAN}${C_BOLD}==================================================
  mr-radio-localization -- image setup
==================================================${C_RESET}
EOF
(( DRY_RUN )) && log_warn "Dry-run mode: no files will be changed."

# ---------------------------------------------------------------------------
# 1. Prerequisites
# ---------------------------------------------------------------------------

log_step "Checking prerequisites"
require_command docker
if ! docker compose version >/dev/null 2>&1; then
  log_error "Docker Compose plugin not found (docker compose ...). Install it and try again."
  exit 1
fi
log_ok "docker + docker compose available"

# ---------------------------------------------------------------------------
# 2. Existing image?
# ---------------------------------------------------------------------------

log_step "Checking for an existing image"
ACTION="build"
if docker image inspect "$IMAGE" >/dev/null 2>&1; then
  created_raw="$(docker image inspect "$IMAGE" --format '{{.Created}}' 2>/dev/null)"
  created_human="$(date -d "$created_raw" '+%Y-%m-%d %H:%M' 2>/dev/null || echo "$created_raw")"
  image_size="$(docker images "$IMAGE" --format '{{.Size}}' 2>/dev/null)"
  log_info "Found ${C_BOLD}$IMAGE${C_RESET}${C_CYAN} -- built $created_human, $image_size${C_RESET}"

  if [[ -n "$FORCE_ACTION" ]]; then
    ACTION="$FORCE_ACTION"
  elif (( NO_CACHE )); then
    ACTION="build"   # an explicit --no-cache means "rebuild", full stop
  elif (( DRY_RUN || ASSUME_YES )); then
    ACTION="build"   # keep -y/-n's prior always-build behavior; use --launch to skip it non-interactively
  else
    echo
    echo "  ${C_GREEN}1)${C_RESET} Launch it now ${C_DIM}(fast -- skips the build)${C_RESET}"
    echo "  ${C_GREEN}2)${C_RESET} Rebuild ${C_DIM}(picks up any code or Dockerfile changes)${C_RESET}"
    echo "  ${C_YELLOW}3)${C_RESET} Cancel"
    read -r -p "Choice [1]: " image_choice || image_choice=""
    case "${image_choice:-1}" in
      1) ACTION="launch" ;;
      2) ACTION="build" ;;
      3|c|C) log_info "Cancelled."; exit 0 ;;
      *) log_warn "Unrecognized choice '$image_choice' -- launching the existing image."; ACTION="launch" ;;
    esac
  fi
elif [[ "$FORCE_ACTION" == "launch" ]]; then
  log_error "No existing $IMAGE image to launch -- run without --launch to build one first."
  exit 1
else
  log_info "No existing image found -- a full build is needed."
fi

if [[ "$ACTION" == "launch" ]]; then
  log_ok "Skipping the build -- launching the existing image."
  # They already chose to launch; go straight to it unless --no-run overrides.
  [[ -z "$RUN_AFTER" ]] && RUN_AFTER="yes"
fi

if [[ "$ACTION" == "build" ]]; then

# ---------------------------------------------------------------------------
# 3. Disk space
# ---------------------------------------------------------------------------

log_step "Checking disk space"
avail_gb=$(( $(df -Pk "$SCRIPT_DIR" | awk 'NR==2 {print $4}') / 1024 / 1024 ))
mount_point="$(df -P "$SCRIPT_DIR" | awk 'NR==2 {print $6}')"
log_info "Free space on $mount_point: ${avail_gb}GB"

if (( avail_gb < MIN_FREE_GB )); then
  log_warn "Less than ${MIN_FREE_GB}GB free. A full build needs real headroom on top of"
  log_warn "the built image's own size (~25GB) -- it has failed mid-build from running"
  log_warn "out of disk before."

  running="$(docker ps -a --filter "ancestor=$IMAGE" -q 2>/dev/null || true)"
  if [[ -n "$running" ]]; then
    log_warn "Containers from the current image still exist:"
    docker ps -a --filter "ancestor=$IMAGE" --format '  {{.ID}}  {{.Status}}  {{.Names}}'
    log_warn "Removing the image tag won't stop them, but you won't be able to start a"
    log_warn "NEW container from it until this build finishes."
  fi

  if confirm "Free up space now (remove the existing $IMAGE image if present, and prune the Docker build cache)?" y; then
    if (( DRY_RUN )); then
      log_info "[DRY-RUN] docker rmi $IMAGE (if present)"
      log_info "[DRY-RUN] docker builder prune -af"
    else
      docker rmi "$IMAGE" >/dev/null 2>&1 || true
      docker builder prune -af
      avail_gb=$(( $(df -Pk "$SCRIPT_DIR" | awk 'NR==2 {print $4}') / 1024 / 1024 ))
      log_info "Free space now: ${avail_gb}GB"
    fi
    log_ok "Reclaimed space."
  else
    log_warn "Continuing anyway -- the build may fail if it runs out of disk mid-way."
  fi
else
  log_ok "${avail_gb}GB free -- should be enough headroom."
fi

# ---------------------------------------------------------------------------
# 4. Build
# ---------------------------------------------------------------------------

log_step "Building the image"
# --progress is a global `docker compose` flag, not a `build` one -- it has
# to come before the subcommand.
#
# --build-arg CACHEBUST: a `RUN git clone` layer only invalidates when this
# instruction's own text changes, not when the remote it clones actually
# moves -- so without this, a normal (non---no-cache) rebuild would silently
# keep serving a stale checkout of mr-radio-localization/UWBPX4Sim even after
# new commits land. Passing a fresh value every time busts just that layer
# (and everything after it) without paying for a full --no-cache rebuild of
# the much more expensive earlier layers (base image, PX4-Autopilot,
# small_gicp, Micro-XRCE-DDS-Agent). --no-cache still forces everything, for
# when the earlier layers themselves need a clean rebuild.
build_arg=(--build-arg "CACHEBUST=$(date +%s)")
compose_args=(--progress=plain build "${build_arg[@]}" app)
(( NO_CACHE )) && compose_args=(--progress=plain build --no-cache "${build_arg[@]}" app)

if (( DRY_RUN )); then
  log_info "[DRY-RUN] (cd \"$SCRIPT_DIR\" && docker compose ${compose_args[*]})"
else
  ( cd "$SCRIPT_DIR" && docker compose "${compose_args[@]}" )
  log_ok "Image built: $IMAGE"
fi

fi # ACTION == build

# ---------------------------------------------------------------------------
# 5. Bind-mount write permissions
# ---------------------------------------------------------------------------

log_step "Granting the container write access to bind-mounted config directories"
# The image's non-root user (jazzy) is baked in at a fixed uid/gid (1001:1001)
# that generally won't match the host user running this script -- so on the
# directories bind-mounted into the container (see docker-compose.yml's
# volumes:), jazzy is neither the owner nor in the owning group, and gets
# only the "other" permission bits. Host directories/files created by a
# normal `mkdir`/editor are typically mode 775/664, whose "other" bits lack
# write -- so the setup GUI's saves (layouts, params.yaml, presets) fail
# with PermissionError despite the volume mount itself working fine.
#
# Fixed the same way as the X11 socket permission below: open up the
# specific host directories the container needs to write into, rather than
# trying to reconcile uids across host and container. o+rwX adds write for
# "other" (matching existing owner/group read-write) without touching
# execute bits on plain files, and applies recursively so both directories
# (for new files) and already-existing files (layout/params saves that
# overwrite in place) end up writable. Done unconditionally (not just on a
# fresh build) so it's also correct when launching an existing image.
gui_write_dirs=(
  "$SCRIPT_DIR/../UWBPX4Sim/config"
  "$SCRIPT_DIR/../UWBPX4Sim/uwb_gazebo_plugin"
)
for d in "${gui_write_dirs[@]}"; do
  if [[ -d "$d" ]]; then
    if (( DRY_RUN )); then
      log_info "[DRY-RUN] chmod -R o+rwX $d"
    else
      chmod -R o+rwX "$d" && log_ok "$(basename "$d") writable by the container" || \
        log_warn "Could not chmod $d -- the setup GUI may fail to save there."
    fi
  fi
done

# ---------------------------------------------------------------------------
# 6. Run
# ---------------------------------------------------------------------------

log_step "Done"

should_run=0
if [[ "$RUN_AFTER" == "yes" ]]; then
  should_run=1
elif [[ "$RUN_AFTER" == "no" ]]; then
  should_run=0
elif confirm "Start the container now (xhost + docker compose run)?" y; then
  should_run=1
fi

if (( ! should_run )); then
  echo
  echo "Next steps:"
  echo "  xhost +local:docker"
  echo "  cd $SCRIPT_DIR && docker compose run --service-ports --rm app"
  echo
  echo "See docker-compose.yml's comments for the extra X11 socket permission"
  echo "step Wayland/XWayland desktops need for QGroundControl/Gazebo windows"
  echo "to appear."
  exit 0
fi

if (( DRY_RUN )); then
  log_info "[DRY-RUN] xhost +local:docker"
  log_info "[DRY-RUN] chmod 777 on the X11 socket matching \$DISPLAY (Wayland/XWayland hosts)"
  log_info "[DRY-RUN] (cd \"$SCRIPT_DIR\" && exec docker compose run --service-ports --rm app)"
  exit 0
fi

log_step "Granting container access to your X server"
if command -v xhost >/dev/null 2>&1; then
  if xhost +local:docker >/dev/null 2>&1; then
    log_ok "xhost +local:docker"
  else
    log_warn "xhost +local:docker failed (no X server / DISPLAY?)."
    log_warn "GUI apps (QGroundControl, Gazebo) may not be able to open a window."
  fi
else
  log_warn "xhost not found -- skipping. GUI apps may not be able to open a window."
fi

# Extra requirement on Wayland/XWayland desktops: see docker-compose.yml's
# comments for why this is needed there and not on classic Xorg sessions.
if [[ -n "${DISPLAY:-}" ]]; then
  x11_num="${DISPLAY#*:}"
  x11_socket="/tmp/.X11-unix/X${x11_num%%.*}"
  if [[ -S "$x11_socket" ]]; then
    chmod 777 "$x11_socket" 2>/dev/null && log_ok "$x11_socket opened for the container" || true
  fi
fi

log_step "Starting the container"
cd "$SCRIPT_DIR"
# --service-ports: `docker compose run` doesn't publish ports: by default
# (unlike `up`) -- without it, the configuration GUI (option 1 in the
# menu) is unreachable from the host browser even though it runs fine.
exec docker compose run --service-ports --rm app
