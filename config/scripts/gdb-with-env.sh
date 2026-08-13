#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/../.."

if [ -f .env ]; then
    set -a
    . ./.env
    set +a
fi

if [ -n "${NCS_TOOLCHAIN_ROOT:-}" ]; then
    PATH="$NCS_TOOLCHAIN_ROOT/opt/zephyr-sdk/gnu/arm-zephyr-eabi/bin:$NCS_TOOLCHAIN_ROOT/opt/bin:$NCS_TOOLCHAIN_ROOT/opt/bin/Scripts:$NCS_TOOLCHAIN_ROOT/bin:$PATH"
    export PATH
fi

if [ -z "${ZEPHYR_GDB:-}" ]; then
    echo "ZEPHYR_GDB is not set. Copy .env.example to .env and set ZEPHYR_GDB." >&2
    exit 1
fi

if [ ! -x "$ZEPHYR_GDB" ]; then
    echo "ZEPHYR_GDB does not point to an executable: $ZEPHYR_GDB" >&2
    exit 1
fi

# OpenOCD does not service the GDB socket until the launch commands that flash
# and verify the image have finished, which takes longer than GDB's 2 s default.
# Without a longer timeout GDB retransmits qSupported, OpenOCD answers every
# copy, and the replies desync ("Remote replied unexpectedly to
# 'vMustReplyEmpty'").
exec "$ZEPHYR_GDB" -iex "set remotetimeout 60" "$@"
