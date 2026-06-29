#!/bin/sh
set -eu

BASE_URL="${BASE_URL:-http://localhost:8080}"

need_cmd() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "missing required command: $1" >&2
        exit 2
    }
}

expect_status() {
    expected="$1"
    method="$2"
    path="$3"
    body="${4:-}"
    if [ -n "$body" ]; then
        code="$(curl -sS -o /tmp/fobos-smoke-body.$$ -w '%{http_code}' \
            -X "$method" -H 'Content-Type: application/json' \
            --data "$body" "$BASE_URL$path")"
    else
        code="$(curl -sS -o /tmp/fobos-smoke-body.$$ -w '%{http_code}' \
            -X "$method" "$BASE_URL$path")"
    fi
    rm -f /tmp/fobos-smoke-body.$$
    if [ "$code" != "$expected" ]; then
        echo "$method $path returned $code, expected $expected" >&2
        exit 1
    fi
}

need_cmd curl

expect_status 200 GET /
expect_status 200 GET /api/status
expect_status 400 POST /api/fft '{"fft_size":"bad"}'
expect_status 400 POST /api/view '{"visible_start_hz":100,"visible_end_hz":50}'
expect_status 404 GET /does-not-exist

echo "HTTP smoke checks passed for $BASE_URL"
