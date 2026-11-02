#include <Wire.h>

#define MPU_ADDR      0x68
#define LOOP_US       4000UL    
#define PRINT_EVERY   10        
#define CAL_SAMPLES   2000      

const float GYRO_SENS = 65.5;   
const float DT        = 0.004;  
const float RAD2DEG   = 57.29577951;
const float DEG2RAD   = 0.01745329252;

const float ROLL_SIGN  =  1.0;
const float PITCH_SIGN = -1.0;
const float YAW_SIGN   = -1.0;

int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
long gyro_cal[3] = {0, 0, 0};

float gyro_p = 0, gyro_q = 0, gyro_r = 0;   
float angle_roll = 0, angle_pitch = 0, angle_yaw = 0;
float angle_roll_acc = 0, angle_pitch_acc = 0;
float gravity_length = 4096.0; 
float vel_z = 0.0, pos_z = 0.0;
bool first_loop = true;
uint8_t print_counter = 0;
unsigned long loop_timer;

bool motors_on = false;
int global_power = 0; 
float pwr[4] = {0, 0, 0, 0}; 

// M1(FD), M2(TD), M3(TI), M4(FI)
const uint8_t MOTOR_PINS[4][4] = {
  {2, 3, 4, 5},       
  {6, 7, 8, 9},       
  {10, 11, 12, 13},   
  {A0, A1, A2, A3}    
};

const uint8_t step_sequence[4] = { B1000, B0100, B0010, B0001 };
unsigned long last_step_time[4] = {0, 0, 0, 0};
int step_index[4] = {0, 0, 0, 0};
int delay_us[4] = {0, 0, 0, 0}; 

int16_t readInt16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void set_gyro_registers() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
}

void gyro_signalen() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);            
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 14);
  while (Wire.available() < 14);
  acc_x = readInt16(); acc_y = readInt16(); acc_z = readInt16();
  readInt16(); 
  gyro_x = readInt16(); gyro_y = readInt16(); gyro_z = readInt16();
}

void processCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'S') {
      motors_on = (Serial.parseInt() == 1);
    } else if (c == 'P') {
      global_power = Serial.parseInt();
    }
  }
}

void calculateMotorSpeeds() {
  if (!motors_on || global_power == 0) {
    for(int i=0; i<4; i++) {
      delay_us[i] = 0;
      pwr[i] = 0;
    }
    return;
  }
  
  // AUMENTO DE SENSIBILIDAD: Ahora no tendrás que inclinar tanto el dron para ver los motores reaccionar.
  float k = 2.0; 
  
  float adj_pitch = angle_pitch * k;
  float adj_roll  = angle_roll * k;

  // LÓGICA RESTAURADA Y CORREGIDA:
  // Si inclinas hacia adelante (Pitch positivo), los motores DELANTEROS (M1, M4) deben acelerar para compensar.
  // Si inclinas a la izquierda (Roll negativo), los motores IZQUIERDOS (M3, M4) deben acelerar.
  
  pwr[0] = global_power + adj_pitch - adj_roll;  // M1 (Frontal Derecho)
  pwr[1] = global_power - adj_pitch - adj_roll;  // M2 (Trasero Derecho)
  pwr[2] = global_power - adj_pitch + adj_roll;  // M3 (Trasero Izquierdo)
  pwr[3] = global_power + adj_pitch + adj_roll;  // M4 (Frontal Izquierdo)

  for (int i=0; i<4; i++) {
    if (pwr[i] < 1.0) pwr[i] = 0.0;     
    if (pwr[i] > 100.0) pwr[i] = 100.0; 
    
    if (pwr[i] > 0) delay_us[i] = map(pwr[i], 1, 100, 5000, 1000);
    else delay_us[i] = 0;
  }
}

void runSteppers() {
  unsigned long now = micros();
  for(int m = 0; m < 4; m++) {
    if (delay_us[m] > 0 && (now - last_step_time[m] >= delay_us[m])) {
      last_step_time[m] = now;
      step_index[m] = (step_index[m] + 1) % 4;
      for(int pin = 0; pin < 4; pin++) {
        digitalWrite(MOTOR_PINS[m][pin], bitRead(step_sequence[step_index[m]], 3 - pin));
      }
    } else if (delay_us[m] == 0) {
      for(int pin = 0; pin < 4; pin++) digitalWrite(MOTOR_PINS[m][pin], LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  
  Wire.begin();
  Wire.setClock(400000);
  
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega2560__)
    Wire.setWireTimeout(3000, true); 
  #endif

  for(int m = 0; m < 4; m++) {
    for(int pin = 0; pin < 4; pin++) {
      pinMode(MOTOR_PINS[m][pin], OUTPUT);
      digitalWrite(MOTOR_PINS[m][pin], LOW);
    }
  }

  set_gyro_registers();
  Serial.println(F("Calibrando... No mover!"));
  delay(1000);

  long acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {
    gyro_signalen();
    gyro_cal[0] += gyro_x; gyro_cal[1] += gyro_y; gyro_cal[2] += gyro_z;
    acc_x_sum += acc_x; acc_y_sum += acc_y; acc_z_sum += acc_z;
    delay(3);
  }
  gyro_cal[0] /= CAL_SAMPLES; gyro_cal[1] /= CAL_SAMPLES; gyro_cal[2] /= CAL_SAMPLES;

  float ax_off = (float)acc_x_sum / CAL_SAMPLES;
  float ay_off = (float)acc_y_sum / CAL_SAMPLES;
  float az_off = (float)acc_z_sum / CAL_SAMPLES;
  gravity_length = sqrt(ax_off * ax_off + ay_off * ay_off + az_off * az_off);

  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega2560__)
    Wire.clearWireTimeoutFlag();
  #endif

  Serial.println(F("Completado!"));
  delay(500);
  loop_timer = micros();
}

void loop() {
  gyro_signalen();

  float p_new = (gyro_x - gyro_cal[0]) / GYRO_SENS;
  float q_new = (gyro_y - gyro_cal[1]) / GYRO_SENS;
  float r_new = (gyro_z - gyro_cal[2]) / GYRO_SENS;
  gyro_p = 0.7 * gyro_p + 0.3 * p_new;
  gyro_q = 0.7 * gyro_q + 0.3 * q_new;
  gyro_r = 0.7 * gyro_r + 0.3 * r_new;

  float ax = (float)acc_x, ay = (float)acc_y, az = (float)acc_z;
  float acc_total = sqrt(ax * ax + ay * ay + az * az);
  if (acc_total > 0.0) {
    angle_roll_acc  =  asin(ay / acc_total) * RAD2DEG;
    angle_pitch_acc = -asin(ax / acc_total) * RAD2DEG;
  }

  if (first_loop) {
    angle_roll = angle_roll_acc; angle_pitch = angle_pitch_acc; angle_yaw = 0;
    first_loop = false;
  }

  float phi = angle_roll * DEG2RAD, tht = angle_pitch * DEG2RAD;
  float sinPhi = sin(phi), cosPhi = cos(phi), tanTht = tan(tht), secTht = 1.0 / cos(tht);

  angle_roll  += (gyro_p + gyro_q * tanTht * sinPhi + gyro_r * tanTht * cosPhi) * DT;
  angle_pitch += (gyro_q * cosPhi - gyro_r * sinPhi) * DT;
  angle_yaw   += (gyro_q * sinPhi * secTht + gyro_r * cosPhi * secTht) * DT;      

  angle_roll  = 0.9996 * angle_roll  + 0.0004 * angle_roll_acc;
  angle_pitch = 0.9996 * angle_pitch + 0.0004 * angle_pitch_acc;

  float sin_tht = sin(tht), cos_tht = cos(tht);
  float z_earth = -ax * sin_tht + ay * sinPhi * cos_tht + az * cosPhi * cos_tht;
  float accel_v_ms2 = -((z_earth - gravity_length) * (9.81 / gravity_length));
  
  vel_z = vel_z * 0.995 + accel_v_ms2 * DT;
  pos_z = pos_z * 0.99  + vel_z * DT;

  processCommands();
  calculateMotorSpeeds(); 

  if (++print_counter >= PRINT_EVERY) {
    print_counter = 0;
    
    Serial.print(F("Roll:")); Serial.print(ROLL_SIGN * angle_roll, 2);
    Serial.print(F(",Pitch:")); Serial.print(PITCH_SIGN * angle_pitch, 2);
    Serial.print(F(",Yaw:")); Serial.print(YAW_SIGN * angle_yaw, 2);
    Serial.print(F(",Alt:")); Serial.println(pos_z, 4); 

    Serial.print(F("M1:")); Serial.print((int)pwr[0]);
    Serial.print(F(",M2:")); Serial.print((int)pwr[1]);
    Serial.print(F(",M3:")); Serial.print((int)pwr[2]);
    Serial.print(F(",M4:")); Serial.println((int)pwr[3]); 
  }

  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega2560__)
    if (Wire.getWireTimeoutFlag()) {
      Wire.clearWireTimeoutFlag();
    }
  #endif

  while (micros() - loop_timer < LOOP_US) {
    runSteppers();
  }
  loop_timer = micros();
}