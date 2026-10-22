#include <Wire.h>   //Include the Wire.h library so we can communicate with the gyro.

int gyro_address;

unsigned long loop_timer;

bool first_angle;

int temperature;

long acc_x, acc_y, acc_z, acc_total_vector[20], acc_av_vector;

int acc_axis[3], gyro_axis[3];

double gyro_pitch, gyro_roll, gyro_yaw;

float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

int cal_int;

double gyro_axis_cal[3];


//Setup routine
void setup(){
  Serial.begin(57600);                                                                  //Start the serial port.
  Wire.begin();                                                                         //Start the wire library as master
  TWBR = 12;                                                                            //Set the I2C clock speed to 400kHz.

  gyro_address = 0x68;                                                                  //Store the gyro address in the variable.

  set_gyro_registers();                                                                 //Set the specific gyro registers.

  while(Serial.available())data = Serial.read();                                        //Empty the serial buffer.
  data = 0;  //Set the data variable back to zero.

  intro()

  if(cal_int != 2000){
      Serial.print("Calibrating the gyro");
      //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
      for (cal_int = 0; cal_int < 2000 ; cal_int ++){                                   //Take 2000 readings for calibration.
        if(cal_int % 125 == 0){
          digitalWrite(12, !digitalRead(12));   //Change the led status to indicate calibration.
          Serial.print(".");
        }
        gyro_signalen();                                                                //Read the gyro output.
        gyro_axis_cal[0] += gyro_axis[1];                                               //Ad roll value to gyro_roll_cal.
        gyro_axis_cal[1] += gyro_axis[2];                                               //Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[2] += gyro_axis[3];                                               //Ad yaw value to gyro_yaw_cal.                                                          //Set digital poort 4, 5, 6 and 7 low.
        delay(3);                                                                       //Wait 3 milliseconds before the next loop.
      }
      Serial.println(".");
      //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
      gyro_axis_cal[0] /= 2000;                                                         //Divide the roll total by 2000.
      gyro_axis_cal[1] /= 2000;                                                         //Divide the pitch total by 2000.
      gyro_axis_cal[2] /= 2000;                                                         //Divide the yaw total by 2000.
    }

    while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
    loop_timer = micros();                                                    //Set the timer for the next loop.
}

//Main program loop
void loop(){
      
      //Let's get the current gyro data.
      gyro_signalen();

      //TODO: compute the angles
      
      if(!first_angle){
        angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
        angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
        first_angle = true;
      }
      else{
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
      }

    while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
    loop_timer = micros();

}


void set_gyro_registers(){
  //Setup the MPU-6050
  Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
  Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
  Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                      //End the transmission with the gyro

  Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
  Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();                                      //End the transmission with the gyro

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search
  Wire.write(0x1B);                                            //Start reading @ register 0x1B
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(gyro_address, 1);                           //Request 1 bytes from the gyro
  while(Wire.available() < 1);                                 //Wait until the 6 bytes are received
  if(Wire.read() != 0x08){                                     //Check if the value is 0x08
    digitalWrite(12,HIGH);                                     //Turn on the warning led
    while(1)delay(10);                                         //Stay in this loop for ever
  }

  Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search
  Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                      //End the transmission with the gyro
}

void gyro_signalen(){
  Wire.beginTransmission(gyro_address);                        //Start communication with the gyro.
  Wire.write(0x3B);                                            //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                      //End the transmission.
  Wire.requestFrom(gyro_address,14);                           //Request 14 bytes from the gyro.
  while(Wire.available() < 14);                                //Wait until the 14 bytes are received.
  
  acc_axis[0] = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the acc_x variable.
  acc_axis[1] = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the acc_y variable.
  acc_axis[2] = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the acc_z variable.
  
  temperature = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the temperature variable.
  
  gyro_axis[0] = Wire.read()<<8|Wire.read();                   //Read high and low part of the angular data.
  gyro_axis[1] = Wire.read()<<8|Wire.read();                   //Read high and low part of the angular data.
  gyro_axis[2] = Wire.read()<<8|Wire.read();                   //Read high and low part of the angular data.

  if(cal_int == 2000){
    gyro_axis[0] -= gyro_axis_cal[1];                            //Only compensate after the calibration.
    gyro_axis[1] -= gyro_axis_cal[2];                            //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[3];                            //Only compensate after the calibration.
  }
}

//Intro subroutine
void intro(){
  Serial.println(F("Insert your intro here"));
  delay(1500);
}
