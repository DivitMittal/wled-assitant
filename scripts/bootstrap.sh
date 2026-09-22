#!/usr/bin/env bash
# Fetch pinned upstream WLED and wire in this project's override/usermod. Idempotent.
set -euo pipefail

WLED_TAG="${WLED_TAG:-v16.0.1}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WLED_DIR="$ROOT/wled"

if [ ! -d "$WLED_DIR/.git" ]; then
  git clone --depth 1 --branch "$WLED_TAG" https://github.com/wled/WLED.git "$WLED_DIR"
fi

actual="$(git -C "$WLED_DIR" describe --tags --exact-match 2>/dev/null || echo unknown)"
if [ "$actual" != "$WLED_TAG" ]; then
  echo "error: wled/ is at '$actual', expected '$WLED_TAG'" >&2
  exit 1
fi
# upstream commits a few CRLF files that git reports as "modified" on checkout; ignore EOL-only noise
if ! git -C "$WLED_DIR" diff --ignore-cr-at-eol --quiet 2>/dev/null; then
  echo "error: upstream WLED checkout has local modifications (this project must not patch core)" >&2
  git -C "$WLED_DIR" diff --ignore-cr-at-eol --stat >&2
  exit 1
fi

ln -sfn ../platformio_override.ini "$WLED_DIR/platformio_override.ini"
(cd "$WLED_DIR" && npm ci --no-audit --no-fund)
echo "ok: WLED $WLED_TAG ready in $WLED_DIR"
