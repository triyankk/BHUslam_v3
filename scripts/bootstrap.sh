#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
[[ "$(uname -m)" == aarch64 ]] || { echo 'This dependency bundle targets Ubuntu 22.04 ARM64.' >&2; exit 1; }
mkdir -p downloads/debs third_party/sysroot third_party/orbbec
sdk=downloads/orbbec-sdk-v2.10.6-arm64.tar.gz
if [[ ! -f "$sdk" ]]; then
  curl -fL --retry 2 -o "$sdk.part" https://github.com/orbbec/OrbbecSDK_v2/releases/download/v2.10.6/OrbbecSDK_v2.10.6_202609291004_575347c_linux_arm64.tar.gz
  mv "$sdk.part" "$sdk"
fi
echo '00fafc2b566ca98feb3ef0242143d830209b09bac5d41c557cf6ec40452c8df9  downloads/orbbec-sdk-v2.10.6-arm64.tar.gz' | sha256sum --check
tar -xzf "$sdk" --strip-components=1 -C third_party/orbbec
(
  cd downloads/debs
  apt-get download cmake cmake-data libjsoncpp25 librhash0 libeigen3-dev libzbar-dev libzbar0 libqrencode-dev libqrencode4 \
    libboost-serialization1.74-dev libboost-serialization1.74.0 libglew-dev libglew2.2 \
    libopencv-dev=4.5.4+dfsg-9ubuntu4 libopencv-calib3d-dev=4.5.4+dfsg-9ubuntu4 \
    libopencv-features2d-dev=4.5.4+dfsg-9ubuntu4 libopencv-highgui-dev=4.5.4+dfsg-9ubuntu4 \
    libopencv-videoio-dev=4.5.4+dfsg-9ubuntu4 libopencv-objdetect-dev=4.5.4+dfsg-9ubuntu4 \
    libopencv-stitching-dev=4.5.4+dfsg-9ubuntu4
)
for package in downloads/debs/*.deb; do dpkg-deb -x "$package" third_party/sysroot; done
if [[ ! -d third_party/ORB_SLAM3 ]]; then
  git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git third_party/ORB_SLAM3
  git -C third_party/ORB_SLAM3 checkout 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
fi
if [[ ! -d third_party/Pangolin ]]; then
  git clone https://github.com/stevenlovegrove/Pangolin.git third_party/Pangolin
  git -C third_party/Pangolin checkout 3667bb76edcc3e4111d2bee7935ce204f04026d9
fi
for compat_patch in "$PWD"/patches/orb-slam3-*.patch; do
  if git -C third_party/ORB_SLAM3 apply --check "$compat_patch" 2>/dev/null; then
    git -C third_party/ORB_SLAM3 apply "$compat_patch"
  elif ! git -C third_party/ORB_SLAM3 apply --reverse --check "$compat_patch" 2>/dev/null; then
    echo "ORB-SLAM3 compatibility patch does not match: $compat_patch" >&2
    exit 1
  fi
done
echo 'Dependencies staged locally. Run: bash scripts/build.sh'
