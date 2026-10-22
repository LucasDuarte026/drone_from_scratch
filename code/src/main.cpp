#include <Arduino.h>
#include <Wire.h>
#include "drone_config.h"
#include "imu_attitude.h"

// ============================================================================
// Appendix B Canonical Global Variables
// ============================================================================
int gyro_address = IMU_I2C_DEFAULT_ADDR;

unsigned long loop_timer = 0;
bool first_angle = false;

int temperature = 0;

long acc_x = 0;
long acc_y = 0;
long acc_z = 0;

int acc_axis[3] = {0, 0, 0};   // 0: X/Roll, 1: Y/Pitch, 2: Z/Yaw
int gyro_axis[3] = {0, 0, 0};  // 0: X/Roll, 1: Y/Pitch, 2: Z/Yaw

double gyro_roll = 0.0;
double gyro_pitch = 0.0;
double gyro_yaw = 0.0;

double gyro_axis_cal[3] = {0.0, 0.0, 0.0}; // Stationary bias offsets

double gyro_roll_input = 0.0;
double gyro_pitch_input = 0.0;
double gyro_yaw_input = 0.0;

float angle_roll_acc = 0.0f;
float angle_pitch_acc = 0.0f;

float angle_roll = 0.0f;
float angle_pitch = 0.0f;
float angle_yaw = 0.0f;

int cal_int = 0;
static uint8_t telemetry_counter = 0;

// ============================================================================
// IMU Register Configuration & Verification
// ============================================================================
void set_gyro_registers() {
  // Check if IMU acknowledges at default address (0x68), fallback to 0x69
  Wire.beginTransmission(gyro_address);
  if (Wire.endTransmission() != 0) {
    gyro_address = IMU_I2C_ALT_ADDR;
    Wire.beginTransmission(gyro_address);
    if (Wire.endTransmission() != 0) {
      Serial.println(F("[ERROR] IMU MPU-6050/6000 not responding at 0x68 or 0x69!"));
      Serial.println(F("[ERROR] Check I2C wiring: SDA -> A4, SCL -> A5, VCC (5V/3.3V), GND."));
      digitalWrite(PIN_LED_WARNING, HIGH);
      while (1) {
        delay(500);
      }
    }
  }

  // 1. Wake up MPU-6050 (PWR_MGMT_1 register 0x6B = 0x00)
  Wire.beginTransmission(gyro_address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // 2. Configure Gyroscope: +/- 500 dps full scale (GYRO_CONFIG register 0x1B = 0x08)
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(IMU_GYRO_FS_500DPS);
  Wire.endTransmission();

  // 3. Configure Accelerometer: +/- 8g full scale (ACCEL_CONFIG register 0x1C = 0x10)
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(IMU_ACCEL_FS_8G);
  Wire.endTransmission();

  // 4. Verify Gyroscope configuration register (0x1B must equal 0x08)
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  unsigned long timeout = millis();
  while (Wire.available() < 1) {
    if (millis() - timeout > 200) {
      Serial.println(F("[ERROR] Timeout reading GYRO_CONFIG register!"));
      digitalWrite(PIN_LED_WARNING, HIGH);
      while (1) delay(10);
    }
  }
  if (Wire.read() != IMU_GYRO_FS_500DPS) {
    Serial.println(F("[ERROR] GYRO_CONFIG verification mismatch!"));
    digitalWrite(PIN_LED_WARNING, HIGH);
    while (1) delay(10);
  }

  // 5. Configure Digital Low Pass Filter: ~43 Hz (CONFIG register 0x1A = 0x03)
  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(IMU_DLPF_43HZ);
  Wire.endTransmission();

  Serial.println(F("[OK] MPU-6050 registers configured & verified successfully."));
}

// ============================================================================
// Read 14-byte Burst from IMU
// ============================================================================
void gyro_signalen() {
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3B); // Start at ACCEL_XOUT_H (0x3B)
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 14);

  uint8_t count = 0;
  while (Wire.available() < 14 && ++count < 250) {
    delayMicroseconds(5);
  }
  if (Wire.available() < 14) {
    return;
  }

  // Accelerometer raw values (0..2)
  acc_axis[0] = (int16_t)(Wire.read() << 8 | Wire.read());
  acc_axis[1] = (int16_t)(Wire.read() << 8 | Wire.read());
  acc_axis[2] = (int16_t)(Wire.read() << 8 | Wire.read());

  // Die temperature
  temperature = (int16_t)(Wire.read() << 8 | Wire.read());

  // Gyroscope raw values (0..2)
  gyro_axis[0] = (int16_t)(Wire.read() << 8 | Wire.read());
  gyro_axis[1] = (int16_t)(Wire.read() << 8 | Wire.read());
  gyro_axis[2] = (int16_t)(Wire.read() << 8 | Wire.read());

  // Assign canonical global acceleration variables with mounting polarity
  acc_x = (long)(acc_axis[0] * IMU_SIGN_X);
  acc_y = (long)(acc_axis[1] * IMU_SIGN_Y);
  acc_z = (long)(acc_axis[2] * IMU_SIGN_Z);
}

// ============================================================================
// Setup Routine
// ============================================================================
void setup() {
  pinMode(PIN_LED_WARNING, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_WARNING, LOW);
  digitalWrite(PIN_LED_STATUS, LOW);

  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 1500);

  Serial.println(F("\n=================================================="));
  Serial.println(F(" Drone Flight Controller - Stage 1: IMU Telemetry"));
  Serial.println(F(" Rate: 250 Hz Loop | Telemetry: 25 Hz Serial"));
  Serial.println(F("=================================================="));

  Wire.begin();
  TWBR = 12; // Fast Mode 400 kHz I2C clock

  set_gyro_registers();

  // Clear incoming serial buffer
  while (Serial.available()) {
    Serial.read();
  }

  // 2000-sample stationary gyroscope bias calibration
  Serial.print(F("Calibrating gyroscope offsets (keep drone motionless)"));
  for (cal_int = 0; cal_int < 2000; cal_int++) {
    if (cal_int % 125 == 0) {
      digitalWrite(PIN_LED_WARNING, !digitalRead(PIN_LED_WARNING));
      Serial.print(F("."));
    }
    gyro_signalen();
    gyro_axis_cal[0] += (gyro_axis[0] * IMU_SIGN_X);
    gyro_axis_cal[1] += (gyro_axis[1] * IMU_SIGN_Y);
    gyro_axis_cal[2] += (gyro_axis[2] * IMU_SIGN_Z);
    delay(3);
  }
  Serial.println(F(" [DONE]"));

  gyro_axis_cal[0] /= 2000.0;
  gyro_axis_cal[1] /= 2000.0;
  gyro_axis_cal[2] /= 2000.0;

  Serial.print(F("Offsets [Roll, Pitch, Yaw]: "));
  Serial.print(gyro_axis_cal[0]);
  Serial.print(F(", "));
  Serial.print(gyro_axis_cal[1]);
  Serial.print(F(", "));
  Serial.println(gyro_axis_cal[2]);

  digitalWrite(PIN_LED_WARNING, LOW);
  digitalWrite(PIN_LED_STATUS, HIGH);

  Serial.println(F("Streaming attitude format: Roll Pitch Yaw"));
  Serial.println(F("--------------------------------------------------"));

  loop_timer = micros();
}

// ============================================================================
// Main Control Loop (Strict 250 Hz / 4000 us Budget)
// ============================================================================
void loop() {
  // 1. Read raw burst from IMU
  gyro_signalen();

  // 2. Compensate stationary bias and convert to body angular rates (deg/s)
  double raw_roll_dps  = ((gyro_axis[0] * IMU_SIGN_X) - gyro_axis_cal[0]) / GYRO_SENSITIVITY_LSB_DPS;
  double raw_pitch_dps = ((gyro_axis[1] * IMU_SIGN_Y) - gyro_axis_cal[1]) / GYRO_SENSITIVITY_LSB_DPS;
  double raw_yaw_dps   = ((gyro_axis[2] * IMU_SIGN_Z) - gyro_axis_cal[2]) / GYRO_SENSITIVITY_LSB_DPS;

  // 3. First-order digital IIR low-pass filter (Eq. 1-3)
  gyro_roll_input  = imu_filter_rate(gyro_roll_input,  raw_roll_dps);
  gyro_pitch_input = imu_filter_rate(gyro_pitch_input, raw_pitch_dps);
  gyro_yaw_input   = imu_filter_rate(gyro_yaw_input,   raw_yaw_dps);

  // 4. Non-linear Euler kinematic rate transformation (Eq. 4-7)
  double d_roll = 0.0, d_pitch = 0.0, d_yaw = 0.0;
  imu_calc_euler_rates(gyro_roll_input, gyro_pitch_input, gyro_yaw_input,
                       angle_roll, angle_pitch,
                       &d_roll, &d_pitch, &d_yaw);

  // 5. Numerical Euler integration over Delta t = 0.004 s
  float angle_roll_gyro  = angle_roll  + (float)(d_roll  * (double)DT_SECONDS);
  float angle_pitch_gyro = angle_pitch + (float)(d_pitch * (double)DT_SECONDS);
  angle_yaw += (float)(d_yaw * (double)DT_SECONDS);

  // 6. Accelerometer tilt angles estimation (Eq. 8-10)
  imu_calc_acc_angles((double)acc_x, (double)acc_y, (double)acc_z,
                      &angle_roll_acc, &angle_pitch_acc);

  // 7. Complementary filter sensor fusion (Eq. 11-12)
  if (!first_angle) {
    angle_roll  = angle_roll_acc;
    angle_pitch = angle_pitch_acc;
    angle_yaw   = 0.0f;
    first_angle = true;
  } else {
    angle_roll  = imu_fuse_angle(angle_roll_gyro,  angle_roll_acc);
    angle_pitch = imu_fuse_angle(angle_pitch_gyro, angle_pitch_acc);
  }

  // 8. Decimated 25 Hz Serial Telemetry Stream (compatible with Serial Plotter and Odometry 3D)
  if (++telemetry_counter >= TELEMETRY_DECIMATION) {
    telemetry_counter = 0;
    Serial.print(angle_roll, 2);
    Serial.print(' ');
    Serial.print(angle_pitch, 2);
    Serial.print(' ');
    Serial.println(angle_yaw, 2);
  }

  // 9. Loop Overrun Watchdog
  if ((micros() - loop_timer) > LOOP_OVERRUN_THRESHOLD_US) {
    digitalWrite(PIN_LED_WARNING, HIGH);
  } else {
    digitalWrite(PIN_LED_WARNING, LOW);
  }

  // 10. Enforce strict 250 Hz (4000 us) loop timing
  while ((micros() - loop_timer) < LOOP_TIME_US);
  loop_timer = micros();
}
