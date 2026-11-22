#!/usr/bin/env bash
set -euo pipefail

# Do not assume USB bus/device numbers survive reboot. Resolve the exact camera
# from sysfs on every poll and require read/write access as the service user.
# The alternate roots are used by the delayed-enumeration fixture only.
vio_wait_seconds="${1:-120}"
vio_wait_cube="${BHU_CUBE_DEVICE:-/dev/serial/by-id/usb-CubePilot_CubeOrange+_36003F000B51333338373339-if00}"
vio_wait_sysfs="${BHU_USB_SYSFS_ROOT:-/sys/bus/usb/devices}"
vio_wait_dev="${BHU_USB_DEV_ROOT:-/dev/bus/usb}"
[[ "$vio_wait_seconds" =~ ^[0-9]+$ ]] && (( vio_wait_seconds >= 1 && vio_wait_seconds <= 600 )) || { echo 'Invalid device wait timeout' >&2; exit 2; }
read -r vio_wait_uptime _ < /proc/uptime
vio_wait_deadline=$(( ${vio_wait_uptime%%.*} + vio_wait_seconds ))
vio_wait_stable=0
read -r vio_wait_boot_id < /proc/sys/kernel/random/boot_id
echo "VIO boot startup: boot_id=$vio_wait_boot_id invocation=${INVOCATION_ID:-manual}"
echo "Waiting up to ${vio_wait_seconds}s for Cube USB and Gemini CPC2963000JJ access."
while :; do
  vio_wait_camera=''
  for vio_wait_entry in "$vio_wait_sysfs"/*; do
    [[ -r "$vio_wait_entry/idVendor" && -r "$vio_wait_entry/idProduct" && -r "$vio_wait_entry/serial" ]] || continue
    read -r vio_wait_vendor < "$vio_wait_entry/idVendor"
    read -r vio_wait_product < "$vio_wait_entry/idProduct"
    read -r vio_wait_serial < "$vio_wait_entry/serial"
    [[ "$vio_wait_vendor" == 2bc5 && "$vio_wait_product" == 0807 && "$vio_wait_serial" == CPC2963000JJ ]] || continue
    [[ -r "$vio_wait_entry/busnum" && -r "$vio_wait_entry/devnum" ]] || continue
    read -r vio_wait_bus < "$vio_wait_entry/busnum"
    read -r vio_wait_device < "$vio_wait_entry/devnum"
    [[ "$vio_wait_bus" =~ ^[0-9]+$ && "$vio_wait_device" =~ ^[0-9]+$ ]] || continue
    printf -v vio_wait_node '%s/%03d/%03d' "$vio_wait_dev" "$((10#$vio_wait_bus))" "$((10#$vio_wait_device))"
    [[ -r "$vio_wait_node" && -w "$vio_wait_node" ]] && vio_wait_camera="$vio_wait_node"
  done
  if [[ -n "$vio_wait_camera" && -r "$vio_wait_cube" && -w "$vio_wait_cube" ]]; then
    ((vio_wait_stable+=1))
    if (( vio_wait_stable >= 3 )); then
      echo "Devices accessible: Cube $vio_wait_cube; Gemini $vio_wait_camera"
      exit 0
    fi
  else
    vio_wait_stable=0
  fi
  read -r vio_wait_uptime _ < /proc/uptime
  if (( ${vio_wait_uptime%%.*} >= vio_wait_deadline )); then
    echo "Timed out: Cube access=$([[ -r "$vio_wait_cube" && -w "$vio_wait_cube" ]] && echo yes || echo no), Gemini access=${vio_wait_camera:-missing}. Check power, USB and udev permissions." >&2
    exit 1
  fi
  sleep .5
done
