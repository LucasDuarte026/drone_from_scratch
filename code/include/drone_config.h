#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

// ============================================================================
// Drone Flight Controller Configuration & Constants
// Stage 1: IMU Attitude Determination
// ============================================================================

// Serial Communication
#define SERIAL_BAUD_RATE            57600
#define TELEMETRY_DECIMATION        10       // 250 Hz / 10 = 25 Hz output rate

// Timing Budget
#define LOOP_TIME_US                4000     // 250 Hz loop period (4000 microseconds)
#define LOOP_OVERRUN_THRESHOLD_US   4050     // Overrun warning threshold
#define DT_SECONDS                  0.004f   // Delta t = 4000 us = 0.004 s

// Pin Assignments
#define PIN_LED_WARNING             12       // D12 indicator / watchdog LED
#define PIN_LED_STATUS              13       // Arduino onboard LED

// IMU Configuration (MPU-6050 / MPU-6000)
#define IMU_I2C_DEFAULT_ADDR        0x68
#define IMU_I2C_ALT_ADDR            0x69
#define IMU_GYRO_FS_500DPS          0x08     // FS_SEL = 1 (+/- 500 dps)
#define IMU_ACCEL_FS_8G             0x10     // AFS_SEL = 2 (+/- 8g)
#define IMU_DLPF_43HZ               0x03     // DLPF_CFG = 3 (~43 Hz bandwidth)
#define GYRO_SENSITIVITY_LSB_DPS    65.5     // 65.5 LSB/(deg/s)

// Axis Mounting Polarity (+1 or -1)
#define IMU_SIGN_X                  (+1)
#define IMU_SIGN_Y                  (+1)
#define IMU_SIGN_Z                  (+1)

// Filter Coefficients
#define LPF_GYRO_PREV               0.7
#define LPF_GYRO_NEW                0.3
#define COMP_FILTER_GYRO            0.9996
#define COMP_FILTER_ACC             0.0004

// Mathematical Constants
#ifndef DEG_TO_RAD
#define DEG_TO_RAD                  0.017453292519943295769236907684886
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG                  57.295779513082320876798154814105
#endif

#endif // DRONE_CONFIG_H
