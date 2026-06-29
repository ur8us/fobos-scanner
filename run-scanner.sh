#!/bin/sh
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR" || exit 1
export LD_LIBRARY_PATH="$DIR/../libfobos-sdr-agile/build-local:$DIR/../local-agile/lib"
exec "$DIR/fobos-scanner" "$@"
