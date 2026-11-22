#!/usr/bin/env bash
set -euo pipefail
wait_script="${1:?wait script required}"
test_root="$(mktemp -d /tmp/bhu-vio-boot.XXXXXX)"
test_pid=''
cleanup() { if [[ -n "$test_pid" ]]; then kill "$test_pid" 2>/dev/null || true; wait "$test_pid" 2>/dev/null || true; fi; rm -rf -- "$test_root"; }
trap cleanup EXIT
export BHU_USB_SYSFS_ROOT="$test_root/sys"
export BHU_USB_DEV_ROOT="$test_root/dev"
export BHU_CUBE_DEVICE="$test_root/cube"
mkdir -p "$test_root/sys/camera" "$test_root/dev/002"
bash "$wait_script" 5 > "$test_root/delayed.log" 2>&1 &
test_pid=$!
sleep .2
kill -0 "$test_pid" # Must not succeed until devices exist.
printf '2bc5\n' > "$test_root/sys/camera/idVendor"
printf '0807\n' > "$test_root/sys/camera/idProduct"
printf 'CPC2963000JJ\n' > "$test_root/sys/camera/serial"
printf '2\n' > "$test_root/sys/camera/busnum"
printf '9\n' > "$test_root/sys/camera/devnum"
touch "$test_root/dev/002/009" "$BHU_CUBE_DEVICE"
wait "$test_pid"; test_pid=''
rg -q 'Gemini .*/002/009' "$test_root/delayed.log" # No hardcoded 002/003.
printf 'WRONG_CAMERA\n' > "$test_root/sys/camera/serial"
if bash "$wait_script" 1 > "$test_root/wrong.log" 2>&1; then echo 'Accepted wrong camera' >&2; exit 1; fi
rg -q 'Timed out' "$test_root/wrong.log"
printf 'CPC2963000JJ\n' > "$test_root/sys/camera/serial"
rm "$BHU_CUBE_DEVICE"
if bash "$wait_script" 1 > "$test_root/missing.log" 2>&1; then echo 'Accepted absent Cube' >&2; exit 1; fi
rg -q 'Cube access=no' "$test_root/missing.log"
if bash "$wait_script" invalid > /dev/null 2>&1; then echo 'Accepted invalid timeout' >&2; exit 1; fi
echo 'PASS: delayed device arrival, changed USB device number, wrong serial, missing Cube, bounded timeout'
