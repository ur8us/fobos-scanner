#!/bin/sh
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
cd "$ROOT" || exit 1
FOBOS_BUILD="${FOBOS_BUILD:-$ROOT/../libfobos-sdr-agile/build-local}"
FOBOS_LOCAL="${FOBOS_LOCAL:-$ROOT/../local-agile}"
export LD_LIBRARY_PATH="$FOBOS_BUILD:$FOBOS_LOCAL/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$DIR/fobos-comp-test" "$@"
