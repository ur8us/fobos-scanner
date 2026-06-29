#!/bin/sh
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR" || exit 1
FOBOS_BUILD="${FOBOS_BUILD:-$DIR/../libfobos-sdr-agile/build-local}"
FOBOS_LOCAL="${FOBOS_LOCAL:-$DIR/../local-agile}"
export LD_LIBRARY_PATH="$FOBOS_BUILD:$FOBOS_LOCAL/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$DIR/fobos-scanner" "$@"
