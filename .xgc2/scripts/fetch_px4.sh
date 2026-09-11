#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=.xgc2/scripts/lib/manifest.sh
source "${SCRIPT_DIR}/lib/manifest.sh"

WORK_DIR="${PX4_FETCH_WORK_DIR:-${PWD}/.work}"
PX4_REPO="$(require_manifest_value px4_repo)"
PX4_TAG="$(require_manifest_value px4_tag)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --repo)
      PX4_REPO="$2"
      shift 2
      ;;
    --tag)
      PX4_TAG="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

PX4_DIR="${WORK_DIR}/PX4-Autopilot"
mkdir -p "${WORK_DIR}"

if [[ ! -d "${PX4_DIR}/.git" ]]; then
  git clone "${PX4_REPO}" "${PX4_DIR}" >&2
fi

git -C "${PX4_DIR}" fetch --tags --prune origin >&2
git -C "${PX4_DIR}" checkout "${PX4_TAG}" >&2
git -C "${PX4_DIR}" submodule update --init --recursive >&2

SITL_GAZEBO="${PX4_DIR}/Tools/sitl_gazebo"
PATCH_DIR="$(cd "${SCRIPT_DIR}/../patches" && pwd)"
if [[ -d "${SITL_GAZEBO}" && -d "${PATCH_DIR}" ]]; then
  shopt -s nullglob
  for patch in "${PATCH_DIR}"/*.patch; do
    echo "Applying $(basename "${patch}") to sitl_gazebo" >&2
    git -C "${SITL_GAZEBO}" apply "${patch}"
  done
  shopt -u nullglob
fi

echo "${PX4_DIR}"
