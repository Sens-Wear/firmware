#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/../.."

if [ -f .env ]; then
    set -a
    . ./.env
    set +a
fi

if [ -z "${ZEPHYR_GDB:-}" ]; then
    echo "ZEPHYR_GDB is not set. Copy .env.example to .env and set ZEPHYR_GDB." >&2
    exit 1
fi

if [ ! -x "$ZEPHYR_GDB" ]; then
    echo "ZEPHYR_GDB does not point to an executable: $ZEPHYR_GDB" >&2
    exit 1
fi

exec "$ZEPHYR_GDB" "$@"
