#!/bin/bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Scheduled performance test for vision-style perception nodes.
# Metrics: init_ms (node + model + publishers ready), first_output_ms (image -> first message).

set -eo pipefail

: "${PERCEPTION_MODULE:?}"
: "${PERCEPTION_NODE:?}"
: "${PERCEPTION_PACKAGES:?}"
: "${PERCEPTION_CONFIG_REL:?}"

INIT_THRESHOLD_MS="${PERCEPTION_INIT_THRESHOLD_MS:-5000}"
INFERENCE_THRESHOLD_MS="${PERCEPTION_INFERENCE_THRESHOLD_MS:-8000}"
PERCEPTION_IMAGE_TOPIC="${PERCEPTION_IMAGE_TOPIC:-/test/image_raw}"
PUBLISH_FRAMES="${PERCEPTION_PERF_PUBLISH_FRAMES:-30}"
TEST_IMAGE_PATH=""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_test_utils.sh"

stop_process_group() {
  local pid="${1:-}"
  [ -n "${pid}" ] || return 0

  # Nodes are launched via setsid, so pid is also process-group id.
  kill -INT "-${pid}" 2>/dev/null || true
  for _ in 1 2 3 4 5 6; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.5
  done
  kill -TERM "-${pid}" 2>/dev/null || true
  sleep 0.5
  kill -KILL "-${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

ARTIFACT_DIR="${SROBOTIS_TEST_ARTIFACT_DIR:-${PWD}}"
METRICS_FILE="${ARTIFACT_DIR}/perception_perf_${PERCEPTION_MODULE}.txt"
INIT_MS=""
FIRST_OUTPUT_MS=""
METRICS_EXTRA=""

metric_label_key() {
  local raw="${1:-default}"
  echo "${raw}" | tr -cs '[:alnum:]_' '_'
}

record_metric_pair() {
  local key="$1"
  local field="$2"
  local value="$3"
  METRICS_EXTRA+="${field}_${key}=${value}"$'\n'
}

write_metrics() {
  mkdir -p "$(dirname "${METRICS_FILE}")"
  {
    echo "module=${PERCEPTION_MODULE}"
    echo "init_ms=${INIT_MS:-}"
    echo "first_output_ms=${FIRST_OUTPUT_MS:-}"
    echo "init_threshold_ms=${INIT_THRESHOLD_MS}"
    echo "inference_threshold_ms=${INFERENCE_THRESHOLD_MS}"
    echo "model=${MODEL_PATH:-}"
    if [ -n "${METRICS_EXTRA}" ]; then
      printf "%s" "${METRICS_EXTRA}"
    fi
  } >"${METRICS_FILE}"
  echo "Metrics written to ${METRICS_FILE}"
}


resolve_model_and_config() {
  local config_rel="${1:-${PERCEPTION_CONFIG_REL}}"
  PKG_CONFIG="$(pkg_share_config "${PERCEPTION_MODULE}" "${config_rel}")"
  if [ ! -f "${PKG_CONFIG}" ]; then
    echo "FAIL: config not found in installed package: ${PKG_CONFIG}" >&2
    exit 1
  fi
  MODEL_PATH="$(grep '^model_path:' "${PKG_CONFIG}" | head -1 | sed 's/^model_path:[[:space:]]*//')"
  MODEL_PATH="${MODEL_PATH//\"/}"
  MODEL_PATH="${MODEL_PATH//\'/}"
  MODEL_PATH="${MODEL_PATH/#\~/$HOME}"
  if [ ! -f "${MODEL_PATH}" ]; then
    echo "FAIL: model not found at ${MODEL_PATH}" >&2
    exit 1
  fi
  TEST_IMAGE_PATH="$(grep '^test_image:' "${PKG_CONFIG}" | head -1 | sed 's/^test_image:[[:space:]]*//' || true)"
  TEST_IMAGE_PATH="${TEST_IMAGE_PATH//\"/}"
  TEST_IMAGE_PATH="${TEST_IMAGE_PATH//\'/}"
  TEST_IMAGE_PATH="${TEST_IMAGE_PATH/#\~/$HOME}"
  if [ -n "${TEST_IMAGE_PATH}" ] && [ ! -f "${TEST_IMAGE_PATH}" ]; then
    local cfg_dir resolved=""
    cfg_dir="$(dirname "${PKG_CONFIG}")"
    if [ -f "${cfg_dir}/${TEST_IMAGE_PATH}" ]; then
      TEST_IMAGE_PATH="${cfg_dir}/${TEST_IMAGE_PATH}"
    fi
  fi
  if [ -n "${TEST_IMAGE_PATH}" ] && [ ! -f "${TEST_IMAGE_PATH}" ]; then
    local resolved=""
    if resolved="$(ensure_perception_test_image "${TEST_IMAGE_PATH}")"; then
      TEST_IMAGE_PATH="${resolved}"
    else
      echo "WARN: test_image not found (${TEST_IMAGE_PATH}), fallback to synthetic frames"
      TEST_IMAGE_PATH=""
    fi
  fi
  return 0
}

run_init_benchmark() {
  local extra_args="${1:-}"
  local label="${2:-default}"
  local config_rel="${3:-${PERCEPTION_CONFIG_REL}}"

  resolve_model_and_config "${config_rel}"

  echo "=== Benchmark init (${label}) ==="
  echo "model=${MODEL_PATH}"

  local start end elapsed node_pid
  start=$(date +%s%N)
  # shellcheck disable=SC2086
  setsid ros2 run "${PERCEPTION_MODULE}" "${PERCEPTION_NODE}" --ros-args \
    -p "config_path:=${PKG_CONFIG}" \
    -p use_camera:=false \
    -p lazy_load:=false \
    -p "image_topic:=${PERCEPTION_IMAGE_TOPIC}" \
    ${extra_args} 2>/dev/null &
  node_pid=$!
  NODE_PID="${node_pid}"

  local ok=0
  local ready_topic="${PERCEPTION_TOPIC:-}"
  local _
  for _ in $(seq 1 40); do
    if ros2 node list 2>/dev/null | grep -q "${PERCEPTION_NODE}"; then
      if [ -n "${ready_topic}" ]; then
        if ros2 topic list 2>/dev/null | grep -Fxq "${ready_topic}"; then
          ok=1
          end=$(date +%s%N)
          break
        fi
      else
        sleep 0.5
        ok=1
        end=$(date +%s%N)
        break
      fi
    fi
    sleep 0.5
  done

  stop_process_group "${node_pid}"
  NODE_PID=""

  if [ "${ok}" -ne 1 ]; then
    echo "FAIL (${label}): node/topic not ready within 20s" >&2
    exit 1
  fi

  elapsed=$(( (end - start) / 1000000 ))
  INIT_MS="${elapsed}"
  local metric_key
  metric_key="$(metric_label_key "${label}")"
  record_metric_pair "${metric_key}" "init_ms" "${elapsed}"
  echo "init_ms (${label}): ${elapsed}"
  if [ "${elapsed}" -gt "${INIT_THRESHOLD_MS}" ]; then
    echo "FAIL (${label}): init ${elapsed}ms > ${INIT_THRESHOLD_MS}ms" >&2
    exit 1
  fi
  echo "PASS (${label}): init within ${INIT_THRESHOLD_MS}ms"
}

run_inference_benchmark() {
  local extra_args="${1:-}"
  local label="${2:-default}"
  local config_rel="${3:-${PERCEPTION_CONFIG_REL}}"

  resolve_model_and_config "${config_rel}"

  local ready_topic="${PERCEPTION_TOPIC:-}"
  if [ -z "${ready_topic}" ]; then
    echo "SKIP inference (${label}): PERCEPTION_TOPIC not set"
    return 0
  fi

  echo ""
  echo "=== Benchmark first output (${label}) ==="
  echo "image_topic=${PERCEPTION_IMAGE_TOPIC} output_topic=${ready_topic}"

  local node_pid pub_pid pub_start end elapsed echo_rc echo_pid
  local echo_file="${TMPDIR:-/tmp}/perception_perf_echo.txt"
  # shellcheck disable=SC2086
  setsid ros2 run "${PERCEPTION_MODULE}" "${PERCEPTION_NODE}" --ros-args \
    -p "config_path:=${PKG_CONFIG}" \
    -p use_camera:=false \
    -p lazy_load:=false \
    -p "image_topic:=${PERCEPTION_IMAGE_TOPIC}" \
    ${extra_args} 2>/dev/null &
  node_pid=$!
  NODE_PID="${node_pid}"

  local ready=0
  local _
  for _ in $(seq 1 40); do
    if ros2 topic list 2>/dev/null | grep -Fxq "${ready_topic}"; then
      ready=1
      break
    fi
    sleep 0.5
  done
  if [ "${ready}" -ne 1 ]; then
    stop_process_group "${node_pid}"
    echo "FAIL (${label}): output topic not ready" >&2
    exit 1
  fi

  rm -f "${echo_file}"
  timeout 30s ros2 topic echo "${ready_topic}" --once \
    >"${echo_file}" 2>/dev/null &
  echo_pid=$!
  ECHO_PID="${echo_pid}"
  sleep 0.2

  pub_start=$(date +%s%N)
  if [ -n "${TEST_IMAGE_PATH}" ]; then
    python3 "${SCRIPT_DIR}/publish_test_image.py" \
      --topic "${PERCEPTION_IMAGE_TOPIC}" \
      --count "${PUBLISH_FRAMES}" \
      --image "${TEST_IMAGE_PATH}" \
      --strict-image &
  else
    python3 "${SCRIPT_DIR}/publish_test_image.py" \
      --topic "${PERCEPTION_IMAGE_TOPIC}" \
      --count "${PUBLISH_FRAMES}" &
  fi
  pub_pid=$!
  PUB_PID="${pub_pid}"

  set +e
  wait "${echo_pid}"
  echo_rc=$?
  set -e
  end=$(date +%s%N)

  wait "${pub_pid}" 2>/dev/null || kill "${pub_pid}" 2>/dev/null || true
  wait "${pub_pid}" 2>/dev/null || true
  PUB_PID=""
  ECHO_PID=""
  stop_process_group "${node_pid}"
  NODE_PID=""

  if [ "${echo_rc}" -ne 0 ] || [ ! -s "${echo_file}" ]; then
    echo "FAIL (${label}): no output on ${ready_topic} within 30s" >&2
    exit 1
  fi

  elapsed=$(( (end - pub_start) / 1000000 ))
  FIRST_OUTPUT_MS="${elapsed}"
  local metric_key
  metric_key="$(metric_label_key "${label}")"
  record_metric_pair "${metric_key}" "first_output_ms" "${elapsed}"
  echo "first_output_ms (${label}): ${elapsed}"
  if [ "${elapsed}" -gt "${INFERENCE_THRESHOLD_MS}" ]; then
    echo "FAIL (${label}): first output ${elapsed}ms > ${INFERENCE_THRESHOLD_MS}ms" >&2
    exit 1
  fi
  echo "PASS (${label}): first output within ${INFERENCE_THRESHOLD_MS}ms"
}

echo "=== ${PERCEPTION_MODULE} Performance Test ==="
echo "thresholds: init<=${INIT_THRESHOLD_MS}ms inference<=${INFERENCE_THRESHOLD_MS}ms"

NODE_PID=""
PUB_PID=""
ECHO_PID=""

cleanup_perf_nodes() {
  if [ -n "${ECHO_PID}" ]; then
    kill "${ECHO_PID}" 2>/dev/null || true
  fi
  if [ -n "${PUB_PID}" ]; then
    stop_node_gracefully "${PUB_PID}" || true
  fi
  if [ -n "${NODE_PID}" ]; then
    stop_process_group "${NODE_PID}" || true
  fi
}
trap cleanup_perf_nodes EXIT INT TERM

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common_setup.sh"

if [ -n "${PERCEPTION_TRACKER_TYPES:-}" ]; then
  for tracker in ${PERCEPTION_TRACKER_TYPES}; do
    run_init_benchmark "-p tracker_type:=${tracker}" "${tracker}" "config/${tracker}.yaml"
    run_inference_benchmark "-p tracker_type:=${tracker}" "${tracker}" "config/${tracker}.yaml"
  done
else
  run_init_benchmark "" "default" "${PERCEPTION_CONFIG_REL}"
  run_inference_benchmark "" "default" "${PERCEPTION_CONFIG_REL}"
fi

echo ""
echo "=== Performance summary ==="
echo "init_ms=${INIT_MS}"
echo "first_output_ms=${FIRST_OUTPUT_MS}"
write_metrics

echo "=== ${PERCEPTION_MODULE} performance tests passed ==="
