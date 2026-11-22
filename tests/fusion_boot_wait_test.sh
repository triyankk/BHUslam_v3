#!/usr/bin/env bash
set -euo pipefail
guard="${1:?guard binary required}"
root="$(mktemp -d /tmp/fusion-boot-ready.XXXXXX)"
child=''
cleanup() { if [[ -n "$child" ]]; then kill "$child" 2>/dev/null || true; wait "$child" 2>/dev/null || true; fi; rm -rf "$root"; }
trap cleanup EXIT
write_status() {
  printf '{"running":true,"updated_at":%s,"cube":{"connected":%s,"armed":%s,"heartbeat_age_ms":%s}}\n' "$1" "$2" "$3" "$4" > "$root/state.tmp"
  mv "$root/state.tmp" "$root/state.json"
}
# A stale ready snapshot, malformed input or an absent Cube must not start VIO.
write_status 1 true false 10
if "$guard" "$root/state.json" 1; then echo 'Accepted stale ready snapshot' >&2; exit 1; fi
printf 'not-json' > "$root/state.json"
if "$guard" "$root/state.json" 1; then echo 'Accepted malformed status' >&2; exit 1; fi
"$guard" "$root/state.json" 6 > "$root/result.log" 2>&1 & child=$!
sleep .3
write_status 2 false false 10
sleep .3
kill -0 "$child"
write_status 3 true true 10
sleep .3
kill -0 "$child" # An armed Cube must not initialize a new estimator.
write_status 4 true false 2000
sleep .3
kill -0 "$child" # Stale heartbeat must not release startup.
write_status 5 true false 10
sleep .3
write_status 6 true false 10
wait "$child"; child=''
rg -q CUBE_STARTUP_READY "$root/result.log"
echo 'PASS: stale/malformed/disconnected/armed/heartbeat guards and fresh readiness'
