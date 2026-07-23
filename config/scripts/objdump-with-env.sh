#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/../.."

if [ -f .env ]; then
    set -a
    . ./.env
    set +a
fi

if [ -z "${ZEPHYR_OBJDUMP:-}" ] && [ -n "${ZEPHYR_GDB:-}" ]; then
    ZEPHYR_OBJDUMP="${ZEPHYR_GDB%-gdb}-objdump"
fi

if [ -z "${ZEPHYR_OBJDUMP:-}" ] || [ ! -x "$ZEPHYR_OBJDUMP" ]; then
    echo "ZEPHYR_OBJDUMP was not found. Set ZEPHYR_GDB or ZEPHYR_OBJDUMP in .env." >&2
    exit 1
fi

exec "$ZEPHYR_OBJDUMP" "$@"
