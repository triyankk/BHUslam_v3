#!/usr/bin/env bash
# Mock ownership failures before any acquisition process can be launched.
set -euo pipefail
test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
test_tmp="$(mktemp -d /tmp/bhu-jt16-ownership.XXXXXX)"
trap 'rm -rf -- "$test_tmp"' EXIT
cat > "$test_tmp/mock-env" <<'MOCK'
systemctl(){
 printf '%s\n' "$*" >> "$JT16_TEST_LOG"
 if [[ "$*" == *is-active* ]]; then echo active; fi
 return 0
}
compgen(){
 if [[ "${1:-}" == -G && "${2:-}" == *CubePilot* ]]; then [[ "$JT16_TEST_CASE" == cube_present ]]; else builtin compgen "$@"; fi
}
flock(){ [[ "$JT16_TEST_CASE" != lock_busy ]]; }
fuser(){ return 0; }
MOCK
for test_case in cube_present lock_busy serial_busy; do
 test_log="$test_tmp/$test_case.commands"
 : > "$test_log"
 if JT16_TEST_CASE="$test_case" JT16_TEST_LOG="$test_log" BASH_ENV="$test_tmp/mock-env" bash "$test_root/bin/jt16-map" "$test_tmp/$test_case" 5 --isolated > "$test_tmp/$test_case.log" 2>&1; then
  echo "FAIL: $test_case should refuse acquisition" >&2; exit 1
 fi
 if [[ "$test_case" == cube_present ]]; then
  [[ ! -s "$test_log" ]] || { echo 'Cube-present case touched services' >&2; exit 1; }
 else
  rg -q -- '--user stop optflow-flight-logger.service' "$test_log"
  rg -q -- '--user start optflow-flight-logger.service' "$test_log"
 fi
 [[ ! -e "$test_tmp/$test_case/driver.log" && ! -e "$test_tmp/$test_case/lio.log" ]] || { echo 'An acquisition process was launched' >&2; exit 1; }
done
echo 'PASS: connected Cube prevents ownership transfer; lock/serial failures restore the previous service; no sensor/backend processes launched in mock cases'
