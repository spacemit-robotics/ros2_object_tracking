#!/bin/bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
export PERCEPTION_MODULE=object_tracking
export PERCEPTION_NODE=object_tracking_node
export PERCEPTION_PACKAGES=object_tracking
export PERCEPTION_CONFIG_REL=config/bytetrack.yaml
export PERCEPTION_TRACKER_TYPES="bytetrack ocsort"
# Tracking perf: init only (node + topic ready). No image/first_output benchmark.
unset PERCEPTION_TOPIC
unset PERCEPTION_IMAGE_TOPIC
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/vision_performance.sh"
