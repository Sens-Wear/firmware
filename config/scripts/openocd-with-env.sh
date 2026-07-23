#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/../.."

if [ -f .env ]; then
    set -a
    . ./.env
    set +a
fi

if [ -z "${OPENOCD:-}" ]; then
    echo "OPENOCD is not set. Copy .env.example to .env and set OPENOCD." >&2
    exit 1
fi

if [ ! -x "$OPENOCD" ]; then
    echo "OPENOCD does not point to an executable: $OPENOCD" >&2
    exit 1
fi

if [ -n "${OPENOCD_SCRIPTS:-}" ]; then
    exec "$OPENOCD" -s "$OPENOCD_SCRIPTS" "$@"
fi

exec "$OPENOCD" "$@"
