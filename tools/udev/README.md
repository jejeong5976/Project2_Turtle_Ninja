# LiDAR device aliases: deployment template

`99-lidars.rules` is intentionally entirely commented out. It installs no aliases
as written. The stationary desk checkpoint uses explicit /dev/serial/by-path paths
and does not require these rules.

The desktop's two CP2102 adapters share VID:PID 10c4:ea60 and serial 0001. Do not use
serial 0001 to distinguish them. The last confirmed desktop socket paths are:

| Role | ID_PATH / by-path basename |
|---|---|
| Front | pci-0000:80:14.0-usb-0:6.4:1.0-port0 |
| Rear | pci-0000:80:14.0-usb-0:6.1:1.0-port0 |

These identify sockets/topology, not unique physical sensors. Verify actual ID_PATH
properties on the target; do not copy desktop identities to Jetson. Reassignment of
sensors to sockets can silently swap front/rear roles, frames and source_id labels.

Before deployment, the user should identify each adapter separately with
`udevadm info --query=property --name=/dev/ttyUSB0` (substitute its current device),
record the properties and physically label sensors and ports. Use distinct serial
rules only if serials are genuinely unique; otherwise fill the path-based pair.
Enable only one pair of rules and remove all placeholders in a deployment copy.
Syntax-check that copy with `udevadm verify /absolute/path/99-lidars.rules` before
user-controlled installation. Verification checks syntax, not correct identity.

After installation/reconnection, verify /dev/lidar_front and /dev/lidar_rear resolve
to different intended devices, access is available through dialout, and assignments
survive reconnect/reboot. Do not activate rules while a driver owns the serial port.
No installation, reload, trigger, permissions change, or physical-device access was
performed during this review.

2026-10-26 review: all repository rules remain disabled. `udevadm verify` passed for
both this file and an enabled-placeholder copy under /tmp; the latter was used only
for syntax checking, never installed. Jetson identity and reconnect tests remain pending.

Separate user-controlled checkpoint:

```bash
git add -- tools/udev/99-lidars.rules tools/udev/README.md
git diff --cached --check
git diff --cached
git commit -m "docs: preserve lidar device identity deployment templates"
```
