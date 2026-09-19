#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT_DIR"

source /opt/ros/humble/setup.bash
source install/setup.bash

PORT="${LIVE_WEB_PORT:-9011}"
SEQUENCE="${LEAGUE_3V3_SEQUENCE:-src/simulator/sample/mock_sequences/league_3v3_center_status.json}"
CONTROL_FILE="${LEAGUE_3V3_CONTROL_FILE:-/tmp/league_3v3_control.jsonl}"
exec env PYTHONPATH=src/simulator python3 -m simulator.start \
  --mode league \
  --offline-decision \
  --keep-to-navi \
  --bypass-is-start \
  --live-view \
  --viewer-config src/simulator/config/league_3v3.yaml \
  --mock-sequence "$SEQUENCE" \
  --control-file "$CONTROL_FILE" \
  --live-web-port "$PORT" \
  "$@" \
  -- runtime_rearm_start_gate:=false
