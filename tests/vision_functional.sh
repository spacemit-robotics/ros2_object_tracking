#!/bin/bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Functional PR test for vision-style perception nodes.
# Required env: PERCEPTION_MODULE, PERCEPTION_NODE, PERCEPTION_PACKAGES,
#               PERCEPTION_TOPIC, PERCEPTION_CONFIG_REL

set -eo pipefail

: "${PERCEPTION_MODULE:?}"
: "${PERCEPTION_NODE:?}"
: "${PERCEPTION_PACKAGES:?}"
: "${PERCEPTION_TOPIC:?}"
: "${PERCEPTION_CONFIG_REL:?}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_test_utils.sh"

echo "=== ${PERCEPTION_MODULE} Functional Test ==="

NODE_PID=""
PUB_PID=""
ECHO_PID=""
cleanup_test_processes() {
  if [ -n "${ECHO_PID}" ]; then
    kill "${ECHO_PID}" 2>/dev/null || true
  fi
  ECHO_PID=""
  stop_node_gracefully "${PUB_PID}"
  PUB_PID=""
  stop_node_gracefully "${NODE_PID}"
  NODE_PID=""
  if [ -n "${MISSING_MODEL_CONFIG:-}" ]; then
    rm -f "${MISSING_MODEL_CONFIG}" 2>/dev/null || true
  fi
}
trap cleanup_test_processes EXIT INT TERM

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_setup.sh"

TEST_INVALID="${SCRIPT_DIR}/test_config_invalid.yaml"
PKG_CONFIG="$(pkg_share_config "${PERCEPTION_MODULE}" "${PERCEPTION_CONFIG_REL}")"
ABS_VALID="${SCRIPT_DIR}/test_config_valid.yaml"
MISSING_MODEL_CONFIG="$(mktemp "${TMPDIR:-/tmp}/perception_missing_model_${PERCEPTION_MODULE}.XXXXXX.yaml")"
MISSING_MODEL_PATH="$(mktemp -u "${TMPDIR:-/tmp}/perception_missing_model_${PERCEPTION_MODULE}.XXXXXX.onnx")"
rm -f "${MISSING_MODEL_PATH}" 2>/dev/null || true
awk -v missing_path="${MISSING_MODEL_PATH}" '
  BEGIN { replaced=0 }
  /^model_path:/ && !replaced { print "model_path: " missing_path; replaced=1; next }
  { print }
' "${ABS_VALID}" >"${MISSING_MODEL_CONFIG}"

if [ ! -f "${PKG_CONFIG}" ]; then
  echo "FAIL: package config not found at ${PKG_CONFIG} (install broken?)" >&2
  exit 1
fi

echo "Test 1: missing model in config (expect node error)"
assert_node_fails_with_error \
  "ros2 run ${PERCEPTION_MODULE} ${PERCEPTION_NODE} --ros-args \
    -p config_path:=${MISSING_MODEL_CONFIG} \
    -p use_camera:=false \
    -p lazy_load:=false" \
  "VisionService::Create failed|Failed to load|runtime_error|Exception"

echo "Test 2: invalid config (expect node error)"
assert_node_fails_with_error \
  "ros2 run ${PERCEPTION_MODULE} ${PERCEPTION_NODE} --ros-args \
    -p config_path:=${TEST_INVALID} \
    -p use_camera:=false \
    -p lazy_load:=false" \
  "YAML|parse|Failed|VisionService::Create failed|runtime_error|Exception"

echo "Test 3: lazy_load node starts and advertises topics"
ros2 run "${PERCEPTION_MODULE}" "${PERCEPTION_NODE}" --ros-args \
  -p "config_path:=${PKG_CONFIG}" \
  -p lazy_load:=true \
  -p use_camera:=false &
NODE_PID=$!
sleep 3

assert_node_starts "${PERCEPTION_NODE}"
assert_topic_exists "${PERCEPTION_TOPIC}"

stop_node_gracefully "${NODE_PID}"
NODE_PID=""

# Tracking modules: PR functional stops at lazy_load + topic advertise (no image inference).
echo "SKIP Test 4: tracking module (no mock-image topic data test)"

echo "=== ${PERCEPTION_MODULE} functional tests passed ==="
