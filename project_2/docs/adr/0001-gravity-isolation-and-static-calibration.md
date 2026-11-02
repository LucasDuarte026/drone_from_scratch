# ADR-0001: Gravity Isolation and Static Accelerometer Bias Calibration

- **Status:** Accepted
- **Date:** 2026-10-28
- **Context:** `project_2/code/src/main.cpp`, `project_2/odometry/sim_3d.py`

---

## 1. Context & Problem Statement

During initial testing of the 6-DoF inertial odometry pipeline, stationary sensor readings exhibited immediate velocity runaways and position drift. When the drone was resting flat on a level desk, raw accelerometer values fluctuated with static offsets, and manual movement followed by coming to rest left residual linear acceleration readings.

Two distinct issues were identified:
1. The IMU's factory offset on the $Z$-axis mixes earth's $1.0g$ gravity vector ($+4096\text{ LSB}$) with sensor bias. On $X$ and $Y$, non-zero resting offsets ($\pm 0.02g$ to $\pm 0.08g$) resulted from soldering stress and temperature.
2. If the user moved or tilted the drone while calibration was executing, erroneous bias values were locked into the baseline, corrupting the complementary filter and odometry forever.

---

## 2. Alternatives Considered

1. **Software-Only Offline Bias Subtraction in Python:**
   - *Pros:* Keeps firmware simple.
   - *Cons:* UART stream transmits raw uncalibrated readings; fails if Python client disconnects or re-syncs; violates low-level hardware abstraction layer (HAL) integrity.
2. **Dynamic High-Pass Filter on Acceleration:**
   - *Pros:* Automatically removes DC bias.
   - *Cons:* Destroys true sustained linear acceleration (e.g. steady climb or translation), making true position estimation impossible.
3. **Dedicated Two-Phase Calibration in Firmware with Visual UI Lockout (Chosen):**
   - *Pros:* Performs 2000-sample averaging directly on ATmega328P at bootup ($4000\,\mu\text{s}$ loop); isolates gravity from the $Z$ channel by subtracting $4096\text{ LSB}$; provides an explicit UI modal banner instructing the user to keep the vehicle completely stationary.

---

## 3. Decision

1. **Firmware Implementation (`code/src/main.cpp`):**
   - The 2000-sample boot calibration accumulates both gyro (`cal_gyro_x`, `cal_gyro_y`, `cal_gyro_z`) and accelerometer offsets (`cal_acc_x`, `cal_acc_y`, `cal_acc_z`).
   - The $Z$-axis baseline offset explicitly subtracts earth gravity:
     $$\text{acc\_bias}[2] = \frac{\sum Acc_z}{2000} - 4096$$
   - Corrected accelerometer values are computed in the primary loop:
     $$Acc_{x,\text{corr}} = Acc_x - \text{acc\_bias}[0]$$
     $$Acc_{y,\text{corr}} = Acc_y - \text{acc\_bias}[1]$$
     $$Acc_{z,\text{corr}} = Acc_z - \text{acc\_bias}[2]$$
2. **Visual Warning Lockout (`odometry/sim_3d.py`):**
   - During calibration, the 3D Pygame HUD displays an amber warning overlay:
     `"CALIBRATING IMU SENSORS - KEEP DEVICE STATIC"`
   - Kinematic position integration remains strictly paused until the calibration phase completes.

---

## 4. Consequences & Trade-offs

### Positive
- True zero-mean acceleration ($0.000g, 0.000g, 1.000g$) when resting level.
- Firmware transmits pre-calibrated physical units ($g$) directly over UART.
- Visual warning prevents operator movement during the crucial 8-second sampling window.

### Negative / Trade-offs
- The drone must be kept completely flat and stationary for the full 2000 samples (~8 seconds) immediately after power-on.
- Temperature drift over prolonged runs may require re-calibration.
