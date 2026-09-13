# IMU Biomechanics Lab — Setup Guide

A real-time sensor dashboard for biomechanics research using ESP32 + HW-131 (MPU-6050).

---

## Project structure

```
imu-lab/
├── public/
│   ├── index.html     — Dashboard UI
│   ├── style.css      — Styles
│   └── app.js         — App logic (Web Serial + simulation)
├── esp32_firmware/
│   └── esp32_imu.ino  — Arduino sketch for the ESP32
├── server.js          — Tiny local dev server (Node.js)
└── README.md          — This file
```

---

## Hardware wiring

```
HW-131 (MPU-6050 breakout)          ESP32 DevKit
─────────────────────────────────────────────────
VCC  ─────────────────────────────►  3.3V
GND  ─────────────────────────────►  GND
SCL  ─────────────────────────────►  GPIO 22
SDA  ─────────────────────────────►  GPIO 21
INT  ──── (optional, leave unconnected for now)
AD0  ──── (leave unconnected → I2C address 0x68)
```

> ⚠️  Use 3.3V only — the HW-131 is NOT 5V tolerant on its logic pins.
> The ESP32's 3.3V pin can supply enough current for the MPU-6050.

---

## Step 1 — Install Arduino IDE & ESP32 support

1. Download Arduino IDE 2.x from https://www.arduino.cc/en/software
2. Open Arduino IDE → File → Preferences
3. In "Additional boards manager URLs" paste:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to Tools → Board → Boards Manager, search **esp32** and install the
   package by Espressif Systems (v2.x recommended).

---

## Step 2 — Install the MPU6050 library

In Arduino IDE → Sketch → Include Library → Manage Libraries:
- Search for **MPU6050** by Electronic Cats
- Click Install

---

## Step 3 — Flash the firmware

1. Open `esp32_firmware/esp32_imu.ino` in Arduino IDE
2. Select your board: Tools → Board → ESP32 Arduino → **ESP32 Dev Module**
   (or whichever variant matches your board — "NodeMCU-32S", "WROOM-32", etc.)
3. Select the port: Tools → Port → select the COM port your ESP32 is on
   - Windows: COMx (e.g. COM3)
   - macOS: /dev/cu.usbserial-xxxx or /dev/cu.SLAB_USBtoUART
   - Linux: /dev/ttyUSB0 or /dev/ttyACM0
4. Press Upload (→ button)
5. Open Serial Monitor at 115200 baud — you should see JSON lines streaming:
   ```json
   {"r":2.31,"p":-1.05,"y":0.00,"ax":0.0401,"ay":-0.0182,"az":0.9988,"t":1234}
   ```

### Troubleshooting upload
- If upload fails, hold the BOOT button on the ESP32 while clicking Upload,
  then release after "Connecting..." appears.
- On Windows you may need the CP2102 or CH340 USB-Serial driver.
  Download from Silicon Labs: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

---

## Step 4 — Run the dashboard

You need Node.js (v14 or newer): https://nodejs.org

```bash
# In the imu-lab folder:
node server.js
```

Then open your browser at: **http://localhost:3000**

> 🔴 Web Serial requires Chrome or Edge. Firefox does not support it.
> The dashboard automatically falls back to simulation mode if no device
> is found or if you're using an unsupported browser.

---

## Step 5 — Connect & record

1. Plug the ESP32 into your computer via USB
2. Open http://localhost:3000 in Chrome or Edge
3. Click **Connect ESP32**
4. A browser dialog will appear — select the ESP32 serial port
5. The dot turns green and data starts flowing
6. (Optional) Click a body part chip to assign the IMU placement
7. Click **Record** to start capturing data
8. Click **Stop** when done
9. Click **↓ Export CSV** to download the session data

---

## CSV output format

Each row is one IMU sample (100 rows = 1 second at 100 Hz):

| Column      | Description                         | Unit    |
|-------------|-------------------------------------|---------|
| `time_s`    | Time since recording start          | seconds |
| `roll_deg`  | Roll angle (rotation around X-axis) | degrees |
| `pitch_deg` | Pitch angle (rotation around Y-axis)| degrees |
| `yaw_deg`   | Yaw angle (rotation around Z-axis)  | degrees |
| `ax_g`      | Acceleration X                      | g       |
| `ay_g`      | Acceleration Y                      | g       |
| `az_g`      | Acceleration Z                      | g       |
| `amag_g`    | Acceleration magnitude √(ax²+ay²+az²)| g      |
| `body_part` | Assigned body part label            | —       |

---

## Sensor calibration (recommended)

The MPU-6050 has factory offsets that cause drift. To calibrate:

1. Place the IMU flat and still on a level surface
2. In Arduino IDE open: File → Examples → MPU6050 → IMU_Zero
3. Upload and run — it outputs calibration offsets to Serial Monitor
4. Copy the 6 offset values into `esp32_imu.ino`:
   ```cpp
   int16_t ax_offset = -1234;  // paste your values here
   int16_t ay_offset =  567;
   int16_t az_offset =  890;
   int16_t gx_offset = -210;
   int16_t gy_offset =   43;
   int16_t gz_offset =  -88;
   ```
5. Re-upload the firmware

---

## Orientation conventions

```
       +Y (pitch up)
        ↑
        │    ┌──────────────┐
        │    │   [CHIP]     │
+Z ─────┼────│              │────── -Z (yaw right)
(up)    │    │              │
        │    └──────────────┘
        ↓              → +X (roll right)
       -Y
```

- **Roll**: rotation around the long axis of the board (X)
- **Pitch**: rotation around the short axis (Y)
- **Yaw**: rotation around the vertical axis (Z) — note: gyro-only, drifts over time

---

## Next steps for your research

### Scaling to 14 sensors
When you have all 14 IMUs, each needs its own ESP32 (or use an ESP32 with
I2C multiplexer like the TCA9548A to support multiple sensors on one bus).
All ESPs can connect to a central laptop over WiFi/UDP for synchronisation.

### Better data storage
For 14 sensors at 100 Hz, consider:
- **Parquet** (via pandas): `df.to_parquet('session.parquet')` — 10× smaller than CSV
- **SQLite**: good for multi-session querying
- **HDF5**: standard for sensor research, stores all sensors in one file

### Feature extraction ideas for ML
Beyond joint angles, strong features for running/jumping:
- Angular velocity from the gyroscope (raw `gx`, `gy`, `gz`)
- Impulse: area under the |A| curve during takeoff
- Phase lag between joints (hip→knee→ankle timing)
- FFT of acceleration signals (movement rhythm, symmetry)
- Quaternion components instead of Euler angles (avoids gimbal lock)
- Time-to-peak and zero-crossing timing per rep

---

## Requirements summary

| Component | Requirement |
|-----------|-------------|
| Browser   | Chrome or Edge (Web Serial API) |
| Node.js   | v14+ (for local server) |
| Arduino IDE | 2.x with ESP32 board support |
| Library   | MPU6050 by Electronic Cats |
| Hardware  | ESP32 DevKit + HW-131 breakout |
