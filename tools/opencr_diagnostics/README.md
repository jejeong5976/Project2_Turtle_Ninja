# OpenCR receive-only status check

This user-run C++ tool listens for five seconds without sending CONFIG, CMD,
torque requests or text probes. It opens the serial descriptor `O_RDONLY`, sets
raw mode at the repository's 1,000,000 host line rate, disables echo/software and
hardware flow control, and saves received bytes to a new file. No ROS stack,
motor configuration, firmware upload or manipulator connection is needed.

Keep motor power off for this initial unknown-firmware check. Opening/closing a
serial device can affect its control lines; "receive-only" describes application
data writes, not a guarantee that opening a port cannot reset a board. The tool
does not explicitly toggle DTR/RTS. It leaves the host tty in raw mode on exit;
restoring an echo-enabled setting could echo continuing telemetry to the board.
It takes an advisory lock and excludes new opens, but cannot detect every already
open non-cooperating reader. Check `fuser DEVICE` first and use an idle device.

The current repository firmware emits STATE before CONFIG, normally with zero
configured joints, and DIAG periodically. Recognizing those packets establishes
protocol compatibility only. USB product/serial strings do not identify the
flashed firmware build; a matching packet also does not establish motor IDs,
signs, steering zeroes, physical feedback validity or odometry accuracy. Silence,
other formats or corrupt packets are reported without declaring firmware failure.

Build on the machine where the program will run, from the repository root:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
  -I firmware/nt_opencr tools/opencr_diagnostics/listen_status.cpp \
  -o /tmp/opencr_listen_status
```

Jetson drive port identified by the user on 2026-10-27:

```text
/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.3:1.0 -> ttyACM0
```

The by-ID name reports `FFFFFFFEFFFF`; uniqueness across two OpenCR boards is not
established. This by-path identity follows the USB socket, not the board if cables
are swapped. Do not change the LiDAR sockets (`2.4.2` and `2.4.4`). The manipulator
OpenCR is disconnected and the full robot launch remains stopped.

User-run capture on Jetson after building the tool there:

```bash
drive_port=/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.3:1.0
fuser "$drive_port"
# Continue only if no PID is listed. fuser exit 1 is expected for an unused port.
capture_root=$(mktemp -d "$HOME/jetson-opencr-passive.XXXXXX")
/tmp/opencr_listen_status "$drive_port" "$capture_root/capture.bin" \
  > "$capture_root/summary.txt" 2>&1
cat "$capture_root/summary.txt"
printf 'Capture directory: %s\n' "$capture_root"
```

Do not use `sudo` or start `robot.launch.py` to resolve a failure. Inspect the
reported error and current port ownership/permissions. Motor IDs in the existing
hardware map remain assumed; this check intentionally does not configure them.

Desktop verification uses PTYs, not a physical serial device:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
  -I firmware/nt_opencr tools/opencr_diagnostics/test_listen_status.cpp \
  -lutil -o /tmp/test_opencr_listen_status
/tmp/test_opencr_listen_status /tmp/opencr_listen_status
```

Tests cover fragmented valid packets, corrupt checksums, silent/unknown streams,
byte-for-byte capture, and absence of bytes sent toward the simulated device.

## First user-run Jetson result

`/home/turtle/jetson-opencr-passive.yj8V9W`, October 27 2026: 27,448 bytes,
502 STATE and 5 DIAG frames over five seconds; no malformed payloads, checksum
errors or nonadvancing state stamps. Board timestamps span 682420–687430 ms.
The reported last joint count is zero, configured and torque flags are clear,
and the IMU flag is set. This is the expected unconfigured wire format, not
evidence that no motors are attached. No motor communication was established by
this capture: the zero error mask and clear torque flag do not independently
read any motor's registers. The IMU flag is initialization status only.

No repeat capture is needed to establish this protocol observation. Identify
actual motor IDs/settings next: the repository's CONFIG handler changes motor
registers, and cannot be used as a read-only discovery request with guessed IDs.
