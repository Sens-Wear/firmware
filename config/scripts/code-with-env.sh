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

exec code --new-window .
