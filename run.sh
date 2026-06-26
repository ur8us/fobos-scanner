#!/bin/sh
DIR="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="$DIR/../libfobos-sdr-agile/build-local:$DIR/../local-agile/lib"
exec "$DIR/fobos-scanner" "$@"
