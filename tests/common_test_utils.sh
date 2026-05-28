#!/bin/bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
# Shared assertion helpers for perception tests.

set -eo pipefail

cleanup_background_nodes() {
  local pattern="${1:-}"
  local pid="${2:-}"

  : "${pattern}"  # Reserved for compatibility with module-specific callers.
  stop_node_gracefully "${pid}"
  sleep 1
}

# Node must exit non-zero and log must match expected_pattern.
assert_node_fails_with_error() {
  local node_cmd="$1"
  local expected_pattern="$2"
  local timeout_sec="${3:-15}"
  local log_file
  log_file="$(mktemp "${TMPDIR:-/tmp}/perception_node_output.XXXXXX.log")"

  set +e
  # shellcheck disable=SC2086
  timeout "${timeout_sec}s" bash -c "${node_cmd}" 2>&1 | tee "${log_file}"
  local exit_code=$?
  set -e

  if [ "${exit_code}" -eq 0 ]; then
    echo "FAIL: command succeeded but failure was expected" >&2
    cat "${log_file}" >&2
    return 1
  fi

  if [ "${exit_code}" -eq 124 ]; then
    echo "FAIL: node hung (timeout after ${timeout_sec}s) instead of exiting with error" >&2
    cat "${log_file}" >&2
    return 1
  fi

  if grep -Eiq "${expected_pattern}" "${log_file}"; then
    echo "PASS: failed with expected error (${expected_pattern})"
    return 0
  fi

  echo "FAIL: exit=${exit_code} but log missing pattern: ${expected_pattern}" >&2
  cat "${log_file}" >&2
  return 1
}

assert_node_starts() {
  local node_pattern="$1"
  local timeout_sec="${2:-10}"

  for _ in $(seq 1 "${timeout_sec}"); do
    if ros2 node list 2>/dev/null | grep -q "${node_pattern}"; then
      echo "PASS: node matching '${node_pattern}' is running"
      return 0
    fi
    sleep 1
  done

  echo "FAIL: node '${node_pattern}' not found within ${timeout_sec}s" >&2
  ros2 node list 2>/dev/null || true
  return 1
}

assert_topic_exists() {
  local topic_name="$1"
  local timeout_sec="${2:-10}"

  for _ in $(seq 1 "${timeout_sec}"); do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic_name}"; then
      echo "PASS: topic '${topic_name}' exists"
      return 0
    fi
    sleep 1
  done

  echo "FAIL: topic '${topic_name}' not found within ${timeout_sec}s" >&2
  ros2 topic list 2>/dev/null || true
  return 1
}

pkg_share_config() {
  local pkg="$1"
  local rel="$2"
  local prefix
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 not in PATH (source common_setup.sh first)" >&2
    exit 1
  fi
  prefix="$(ros2 pkg prefix "${pkg}")"
  echo "${prefix}/share/${pkg}/${rel}"
}

PERCEPTION_TEST_IMAGE_BASE_URL="${PERCEPTION_TEST_IMAGE_BASE_URL:-https://archive.spacemit.com/spacemit-ai/model_zoo/assets/image}"

# Download yaml test_image from model_zoo assets when missing locally.
ensure_perception_test_image() {
  local image_path="$1"
  local image_name dest_dir download_url tmp_file

  [ -n "${image_path}" ] || return 1
  if [ -f "${image_path}" ]; then
    echo "${image_path}"
    return 0
  fi

  image_name="$(basename "${image_path}")"
  dest_dir="$(dirname "${image_path}")"
  download_url="${PERCEPTION_TEST_IMAGE_BASE_URL}/${image_name}"

  if ! command -v curl >/dev/null 2>&1 && ! command -v wget >/dev/null 2>&1; then
    echo "WARN: test_image missing and curl/wget unavailable (${download_url})" >&2
    return 1
  fi

  mkdir -p "${dest_dir}" || return 1
  tmp_file="$(mktemp "${dest_dir}/.${image_name}.XXXXXX")"

  echo "INFO: downloading test_image from ${download_url} -> ${image_path}"
  if command -v curl >/dev/null 2>&1; then
    if ! curl -fsSL --connect-timeout 15 --max-time 120 -o "${tmp_file}" "${download_url}"; then
      rm -f "${tmp_file}"
      echo "WARN: failed to download test_image from ${download_url}" >&2
      return 1
    fi
  elif ! wget -q -T 120 -O "${tmp_file}" "${download_url}"; then
    rm -f "${tmp_file}"
    echo "WARN: failed to download test_image from ${download_url}" >&2
    return 1
  fi

  mv "${tmp_file}" "${image_path}"
  echo "${image_path}"
  return 0
}

stop_node_gracefully() {
  local pid="${1:-}"
  [ -n "${pid}" ] || return 0
  kill -INT "${pid}" 2>/dev/null || true
  for _ in 1 2 3 4 5 6; do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.5
  done
  kill "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}
