> **Project end goal:** Build a drone that autonomously flies a guided mission without GPS while detecting and avoiding obstacles along the way.
> Camera inspection supports usable visual input and repeatable sensor setup for that goal. See the [shared project goal](../PROJECT_GOAL.md).

# Camera field-of-view and QR preview

The current recording boot service starts automatically for two minutes and
does not use QR. QR inspection below remains available for earlier VIO workflows.

`bin/bhu preview` serves a local-only preview at http://127.0.0.1:8767.
The native C++ tool displays the complete rectified 640x400 left and right
infrared frames used by the recorder. The QR scanner runs on the entire left
frame before preview labels/JPEG compression. It reports exact `test_vio`
recognition separately from a QR that decodes to a different payload.

This mode does not record, run SLAM or open the Cube connection. Stop the
recorder before starting preview: one process owns the camera. The preview
restores reversible camera properties when stopped and defaults to a 30-minute
time limit. Do not start a recording while the preview owns the camera.

The preview is running as a transient user unit `bhu-camera-preview.service`.
Stop it before recording:

```bash
systemctl --user stop bhu-camera-preview.service
systemctl --user start bhu-vio-record.service
```

For a browser on a different computer, an authenticated SSH tunnel can carry
the local-only feed:

```bash
ssh -L 8767:127.0.0.1:8767 atas@JETSON_IP
```

Then open http://127.0.0.1:8767 on that computer. No unauthenticated LAN feed
was enabled. The initial attempt to bind an unauthenticated stream on all
interfaces was rejected by automatic approval review; local-only operation
was used instead.

During the 2026-10-25 phone check, the phone and complete QR were inside both
IR views. The pattern was dim with a narrow light border. No QR payload could
be decoded from a saved preview frame, including normalization/CLAHE/upscaling
diagnostics. This demonstrates that software ROI clipping was not the issue
in that view, but does not establish the sole cause of the decoding failure.
Try the supplied plain `assets/test_vio.png` at a larger size with its full
white border, or print `assets/test_vio.svg`. Screen appearance in IR can differ
from the visible image seen by a person. Do not assume an undecoded QR has the
correct payload; the required value is exactly `test_vio`.

Native diagnostic CLI: `build/main/bin/qr_inspect IMAGE`; its four methods
inspect raw/normalized/CLAHE/upscaled grayscale. These diagnostics do not
change the recorder's production QR detector.
