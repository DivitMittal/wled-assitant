#!/usr/bin/env bash
# Build (and optionally flash) the firmware. Usage: scripts/build.sh [upload|monitor-free args...]
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ENV_NAME="wled_assitant_s3_supermini"

"$ROOT/scripts/bootstrap.sh"
cd "$ROOT/wled"
npm run build
if [ "${1:-}" = "upload" ]; then
  shift
  pio run -e "$ENV_NAME" -t upload "$@"
else
  pio run -e "$ENV_NAME" "$@"
fi
ls -l "$ROOT/wled/build_output/release/" 2>/dev/null || true
