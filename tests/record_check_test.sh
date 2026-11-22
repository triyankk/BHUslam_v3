#!/usr/bin/env bash
set -euo pipefail
checker="${1:?checker path required}"
fixture_root="$(mktemp -d /tmp/bhu-record-check.XXXXXX)"
trap 'rm -rf -- "$fixture_root"' EXIT
mkdir -p "$fixture_root"/{left,right,native_left,native_right}
printf '%s\n' '{"diagnostic":true,"width":2,"height":2,"complete":true,"protocol_duration_complete":false}' > "$fixture_root/recording.json"
printf 'timestamp_s,left,right,left_us,right_us,left_index,right_index\n1.0,left/0.pgm,right/0.pgm,1000000,1000000,5,5\n1.033,left/1.pgm,right/1.pgm,1033000,1033000,6,6\n' > "$fixture_root/frames.csv"
for side in left right native_left native_right; do for index in 0 1; do printf 'P5\n2 2\n255\nabcd' > "$fixture_root/$side/$index.pgm"; done; done
for stream in accel gyro; do
  printf 'timestamp_s,x,y,z\n0.99,0,0,9.81\n1.01,0,0,9.81\n1.04,0,0,9.81\n' > "$fixture_root/$stream.csv"
  printf 'device_us,system_us,host_arrival_us,index,x,y,z\n990000,1990000,2990000,1,0,0,9.81\n1010000,2010000,3010000,2,0,0,9.81\n1040000,2040000,3040000,3,0,0,9.81\n' > "$fixture_root/${stream}_native.csv"
done
"$checker" "$fixture_root" > /dev/null
rg -q '"integrity_pass": true' "$fixture_root/recording-check.json"
rg -q 'IMU gap exceeds' "$fixture_root/recording-check.json"
rg -q 'Full 10/60/10-second protocol' "$fixture_root/recording-check.json"
rm "$fixture_root/native_left/1.pgm"
if "$checker" "$fixture_root" > /dev/null 2>&1; then echo 'Accepted missing native image' >&2; exit 1; fi
printf 'P5\n2 2\n255\nabcd' > "$fixture_root/native_left/1.pgm"
printf '0.98,0,0,9.81\n' >> "$fixture_root/gyro.csv"
if "$checker" "$fixture_root" > /dev/null 2>&1; then echo 'Accepted nonmonotonic IMU' >&2; exit 1; fi
echo 'PASS: valid lossless recording, gap/protocol warnings, missing native image and invalid IMU rejected'
