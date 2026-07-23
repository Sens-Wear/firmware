#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/../.."

if [ -f .env ]; then
    set -a
    . ./.env
    set +a
fi

if [ -z "${ZEPHYR_NM:-}" ] && [ -n "${ZEPHYR_GDB:-}" ]; then
    ZEPHYR_NM="${ZEPHYR_GDB%-gdb}-nm"
fi

if [ -z "${ZEPHYR_NM:-}" ] || [ ! -x "$ZEPHYR_NM" ]; then
    echo "ZEPHYR_NM was not found. Set ZEPHYR_GDB or ZEPHYR_NM in .env." >&2
    exit 1
fi

exec "$ZEPHYR_NM" "$@"
