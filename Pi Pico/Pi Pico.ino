/*
X-Axis sensitivity offset value 1.18
Y-Axis sensitivity offset value 1.18
Z-Axis sensitivity offset value 1.14
Mag Calibration: Wave device in a figure eight until done!
mag x min/max:
-180
377
mag y min/max:
0
417
mag z min/max:
-49
484
Mag Calibration done!
AK8963 mag biases (mG)
172.77, 367.91, 369.85
AK8963 mag scale (mG)
0.89, 1.19, 0.96
Mag Factory Calibration Values: 
X-Axis sensitivity offset value 1.18
Y-Axis sensitivity offset value 1.18
Z-Axis sensitivity offset value 1.14
< calibration parameters >
accel bias [g]: 
-271.60, 20.58, -0.08
gyro bias [deg/s]: 
-0.44, 0.13, -0.40
mag bias [mG]: 
172.77, 367.91, 369.85
mag scale []: 
0.89, 1.19, 0.96
*/

#include <MPU9250.h>
#include <Wire.h>

#define PICO_I2C_ADRESS 0x09

MPU9250 mpu;

double gyroid[3];
double acceleroid[3];

void sendData();
void gyroData();

void setup()
{
  pinMode(20, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(18, OUTPUT);
  Wire.begin();
  Wire1.begin(PICO_I2C_ADRESS);
  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(1000);
      }
  }
  digitalWrite(20, LOW);
  digitalWrite(19, LOW);
  digitalWrite(18, HIGH);
  Serial.println("Accel Gyro calibration will start in 3sec.");
  mpu.verbose(true);
  delay(3000);
  mpu.calibrateAccelGyro();
  digitalWrite(20, LOW);
  digitalWrite(19, HIGH);
  digitalWrite(18, HIGH);

  Serial.println("Please Wave device in a figure eight until done.");
  mpu.calibrateMag();

  print_calibration();
  mpu.verbose(false);
  digitalWrite(20, LOW);
  digitalWrite(19, HIGH);
  digitalWrite(18, LOW);
  Wire1.onRequest(sendData);
}

void loop() {
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
        gyroData();
        prev_ms = millis();
    }
  }
}


void sendData()
{
  int mainGyro = (int)gyroid[0];
  
  byte gy1 = 0;
  byte gy2 = 0;

  if(mainGyro > 0)
    gy1 = mainGyro;

  if(mainGyro < 0)
    gy2 = -mainGyro;

  // gyro
  Wire1.write(gy1);
  Wire1.write(gy2); 
  // speed
  Wire1.write(0x00);
  Wire1.write(0x00); 
  // old gyro
  Wire1.write(0x00);
  Wire1.write(0x00); 
}

void gyroData() {
  gyroid[0] = mpu.getYaw();
  gyroid[1] = mpu.getPitch();
  gyroid[2] = mpu.getRoll();
  Serial.print("Yaw, Pitch: ");
  Serial.print(mpu.getYaw(), 2);
  Serial.print(", ");
  Serial.println(mpu.getPitch(), 2);
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}