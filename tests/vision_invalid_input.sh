#!/bin/bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Invalid-input PR test for vision-style perception nodes.

set -eo pipefail

: "${PERCEPTION_MODULE:?}"
: "${PERCEPTION_NODE:?}"
: "${PERCEPTION_PACKAGES:?}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_test_utils.sh"

echo "=== ${PERCEPTION_MODULE} Invalid Input Test ==="

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_setup.sh"

TEST_INVALID="${SCRIPT_DIR}/test_config_invalid.yaml"
ABS_VALID="${SCRIPT_DIR}/test_config_valid.yaml"
MISSING_CONFIG="$(mktemp -u "${TMPDIR:-/tmp}/perception_missing_config_${PERCEPTION_MODULE}.XXXXXX.yaml")"
rm -f "${MISSING_CONFIG}" 2>/dev/null || true
MISSING_MODEL_CONFIG="$(mktemp "${TMPDIR:-/tmp}/perception_missing_model_${PERCEPTION_MODULE}.XXXXXX.yaml")"
MISSING_MODEL_PATH="$(mktemp -u "${TMPDIR:-/tmp}/perception_missing_model_${PERCEPTION_MODULE}.XXXXXX.onnx")"
rm -f "${MISSING_MODEL_PATH}" 2>/dev/null || true
awk -v missing_path="${MISSING_MODEL_PATH}" '
  BEGIN { replaced=0 }
  /^model_path:/ && !replaced { print "model_path: " missing_path; replaced=1; next }
  { print }
' "${ABS_VALID}" >"${MISSING_MODEL_CONFIG}"

cleanup_invalid_tmp() {
  if [ -n "${MISSING_MODEL_CONFIG:-}" ]; then
    rm -f "${MISSING_MODEL_CONFIG}" 2>/dev/null || true
  fi
}
trap cleanup_invalid_tmp EXIT INT TERM

echo "Test 1: invalid config"
assert_node_fails_with_error \
  "ros2 run ${PERCEPTION_MODULE} ${PERCEPTION_NODE} --ros-args \
    -p config_path:=${TEST_INVALID} \
    -p use_camera:=false \
    -p lazy_load:=false" \
  "YAML|parse|Failed|VisionService::Create failed|runtime_error|Exception"

echo "Test 2: missing config file path"
assert_node_fails_with_error \
  "ros2 run ${PERCEPTION_MODULE} ${PERCEPTION_NODE} --ros-args \
    -p config_path:=${MISSING_CONFIG} \
    -p use_camera:=false \
    -p lazy_load:=false" \
  "VisionService::Create failed|Failed to load|No such file|runtime_error|Exception"

echo "Test 3: missing model weights"
assert_node_fails_with_error \
  "ros2 run ${PERCEPTION_MODULE} ${PERCEPTION_NODE} --ros-args \
    -p config_path:=${MISSING_MODEL_CONFIG} \
    -p use_camera:=false \
    -p lazy_load:=false" \
  "VisionService::Create failed|Failed to load|runtime_error|Exception"

echo "Test 4: invalid score_threshold (expect rejection or startup failure)"
LOG_FILE="$(mktemp "${TMPDIR:-/tmp}/perception_threshold_${PERCEPTION_MODULE}.XXXXXX.log")"
set +e
timeout 10s ros2 run "${PERCEPTION_MODULE}" "${PERCEPTION_NODE}" --ros-args \
  -p score_threshold:=-1.0 \
  -p use_camera:=false \
  -p lazy_load:=true 2>&1 | tee "$LOG_FILE"
TH_RC=$?
set -e
if [ "$TH_RC" -eq 124 ]; then
  echo "FAIL: node hung (timeout) for invalid score_threshold" >&2
  exit 1
elif grep -Eiq 'score_threshold.*(invalid|out of range|must be between)|threshold.*(invalid|out of range)' "$LOG_FILE"; then
  echo "PASS: invalid score_threshold rejected"
elif [ "$TH_RC" -ne 0 ]; then
  echo "PASS: node exited with error for invalid score_threshold"
else
  echo "FAIL: node accepted score_threshold=-1.0 without validation" >&2
  exit 1
fi

echo "=== ${PERCEPTION_MODULE} invalid input tests passed ==="
