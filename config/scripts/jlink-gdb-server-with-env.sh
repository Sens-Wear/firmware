#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/../.."

if [ -f .env ]; then
    set -a
    . ./.env
    set +a
fi

if [ -z "${JLINK_GDB_SERVER:-}" ]; then
    for candidate in JLinkGDBServerCLExe JLinkGDBServerCL JLinkGDBServer; do
        if command -v "$candidate" >/dev/null 2>&1; then
            JLINK_GDB_SERVER="$(command -v "$candidate")"
            break
        fi
    done
fi

if [ -z "${JLINK_GDB_SERVER:-}" ]; then
    echo "J-Link GDB server was not found. Set JLINK_GDB_SERVER in .env." >&2
    exit 1
fi

if [ ! -x "$JLINK_GDB_SERVER" ]; then
    echo "JLINK_GDB_SERVER does not point to an executable: $JLINK_GDB_SERVER" >&2
    exit 1
fi

exec "$JLINK_GDB_SERVER" "$@"
