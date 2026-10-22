## 1. Foundation & Architecture Alignment
- [x] 1.1 Resolve open architectural questions via `/grill-me` (PlatformIO multi-env, Appendix B globals + testable headers, Axis polarity macros, 57600 bps @ 25 Hz decimation + 4050 us LED watchdog)
- [x] 1.2 Encapsulate firmware in `code/` (`code/src/` and `code/include/`), create `platformio.ini` with `[env:uno]` / `[env:stage1_imu]` (Arduino Uno), and author `code/include/drone_config.h` and `code/include/imu_attitude.h`

## 2. Core IMU Driver & Signal Processing (Stage 1)
- [x] 2.1 Fix compilation and array indexing bugs from `example_01.ino` (`data` declaration, `intro();`, `0..2` indexing in `setup()` and `gyro_signalen()`)
- [x] 2.2 Implement gyro scaling ($\pm 500\text{ dps} \rightarrow 65.5\text{ LSB/deg/s}$) and first-order IIR digital low-pass filter ($0.7 / 0.3$, Eq. 1–3)
- [x] 2.3 Implement Euler angle kinematic differential equations and numerical integration ($\Delta t = 0.004\text{ s}$, Eq. 4–7)
- [x] 2.4 Implement accelerometer gravity vector projection and `asin()` pitch/roll estimation (Eq. 8–10) with numerical domain clamping
- [x] 2.5 Implement complementary filter fusion ($\alpha = 0.9996$, $1 - \alpha = 0.0004$, Eq. 11–12) with first-sample initialization

## 3. Verification & Documentation Synchronization
- [x] 3.1 Verify $4000\,\mu\text{s}$ loop timing constraint and Serial output format (`Roll Pitch Yaw` @ 25 Hz)
- [x] 3.2 Synchronize `docs/` SDD documentation, `PROJECT.md`, `README.md`, and update Graphify knowledge graph
