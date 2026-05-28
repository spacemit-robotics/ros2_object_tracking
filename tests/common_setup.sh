#!/bin/bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Setup environment for perception tests (no build, use pre-built staging).
# Usage: source tests/common_setup.sh

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_test_utils.sh"

source_ros() {
  local ros_setup="/opt/ros/humble/setup.bash"
  if [ ! -f "${ros_setup}" ]; then
    echo "ERROR: ROS 2 Humble not found: ${ros_setup}" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  set -u
}

source_sdk_env() {
  local root="${SPACEMIT_SDK_ROOT:-${SROBOTIS_ROOT:-${REPO_ROOT:-}}}"
  if [ -z "${root}" ]; then
    # Walk up from this script until we find an SDK marker (build/envsetup.sh).
    local dir="${SCRIPT_DIR}"
    while [ "${dir}" != "/" ] && [ -n "${dir}" ]; do
      if [ -f "${dir}/build/envsetup.sh" ]; then
        root="${dir}"
        break
      fi
      dir="$(dirname "${dir}")"
    done
  fi

  if [ -z "${root}" ] || [ ! -d "${root}" ]; then
    echo "ERROR: SDK root not found. Set SPACEMIT_SDK_ROOT or run from inside an SDK checkout." >&2
    exit 1
  fi
  export SPACEMIT_SDK_ROOT="${root}"

  if [ ! -f "${root}/build/envsetup.sh" ]; then
    echo "WARNING: envsetup.sh not found under ${root}; libvision.so may be missing" >&2
  else
    set +eu
    # shellcheck disable=SC1090
    source "${root}/build/envsetup.sh" || {
      echo "WARNING: envsetup.sh returned non-zero; continuing" >&2
    }
    set -e
  fi
}

source_staging() {
  local root="${SPACEMIT_SDK_ROOT:-}"
  if [ -z "${root}" ]; then
    echo "ERROR: SPACEMIT_SDK_ROOT not set" >&2
    exit 1
  fi

  local staging_setup="${root}/output/staging/setup.bash"
  if [ ! -f "${staging_setup}" ]; then
    echo "ERROR: staging setup not found: ${staging_setup}" >&2
    echo "Please build the packages first" >&2
    exit 1
  fi

  set +u
  # shellcheck disable=SC1090
  source "${staging_setup}"
  set -u
}

echo "=== Setting up perception test environment ==="

source_ros
source_sdk_env
source_staging

echo "=== Environment ready ==="
