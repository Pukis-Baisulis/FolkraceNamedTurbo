// dip config
// 0. Motors 
// 1. Serial
// 2. Acceleration 
// 3. Braking 
// 4. Dynamic kP
// 5. Delay 


// libraries
#include <Wire.h>
#include <VL53L0X.h>
#include "MPU9250.h"


// pinout
#define xshut1 4
#define xshut2 7
#define xshut3 8
#define AIN_1 9
#define AIN_2 10
#define BIN_1 6
#define BIN_2 5
#define mot_en 11


// operation modes
//#define debug_mode 1 //prints way more data

// initialisation things 
#define serial_baud 115200
#define vlx_refresh_rate 20000//default 20000us (20ms) may be useful for overclocking

// devices
VL53L0X VLX_1;
VL53L0X VLX_2;
VL53L0X VLX_3; 
MPU9250 mpu;


// constants
#define start_delay 5000

#define breaking_cof 0.08
#define accel_cof 0.8 

#define max_spd 255
#define min_spd 70
#define avoid_spd 180
#define def_drive_spd 200

#define max_sensor_range 1200
#define min_sensor_range 40

#define minimal_front_distance 150
#define avoid_dist 80
#define breaking_dist 300
#define reaccel_dist 80


// variables
bool dips[6];
bool vlx[3];
int distance[3];
int gyro[3];
int accelg[3];
int spds[2];
int speed; // front speed in m/s
unsigned long milk_0; // for start delay and measurements 
unsigned long milk_1; // for acceleration
bool avoiding = false;
int accel = 0;



// functions
void init_all(); //also reading the bs from dips
void sensors();
void drive(int spdA, int spdB);
void not_moving();
int emulate_front();
void dipsaliai();


void setup() 
{
  // put your setup code here, to run once
  milk_0 = millis();
  init_all();
  if(dips[5])
  {
    while(millis()-milk_0 <= start_delay)
    {
      digitalWrite(LED_BUILTIN, LOW);
    }    
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() 
{
  sensors();
}

void sensors()
{
  if(vlx[0])
    distance[0]=VLX_1.readRangeContinuousMillimeters();//vlx1 left
  else
    distance[0]=0;
    
  if(distance[0] <=min_sensor_range)
    distance[0]=1;    
  if(distance[0] >=max_sensor_range)
    distance[0]=max_sensor_range;
  if(distance[0] <= reaccel_dist)
    milk_1 = millis();
        

  if(vlx[2])
    distance[2]=VLX_3.readRangeContinuousMillimeters();// vlx3 right
  else
   distance[2]=0;
   
  if(distance[2] <=min_sensor_range)
    distance[2]=1;    
  if(distance[2] >=max_sensor_range)
    distance[2]=max_sensor_range;
  if(distance[2] <= reaccel_dist)
    milk_1 = millis();   


  if(vlx[1])
    distance[1]=VLX_2.readRangeContinuousMillimeters();//vlx2 mid
  else 
    distance[1]=emulate_front();

  if(distance[1] <=min_sensor_range)
    distance[1]=1; 
  if(distance[1] >=max_sensor_range)
    distance[1]=max_sensor_range;    
  if(distance[2] <= avoid_dist && !avoiding)
    avoid(); 
  
  

  /*if(dips[1])
  {
    Serial.print("Dist_1:  ");
    Serial.print(distance[0]);
    Serial.print("  Dist_2:  ");
    Serial.print(distance[1]);
    Serial.print("  Dist_3:  ");
    Serial.print(distance[2]);
    Serial.print("  Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
  }*/
}

void drive(int spdA, int spdB)
{

  

  //accel
  if(dips[2] && !avoiding)// uses milk_1
  {
    accel = (millis()-milk_1)*accel_cof;
    accel = constrain(accel, 0, 100);

    spdA = spdA*accel/100;
    spdB = spdB*accel/100;
  }  

  //braking
  if(dips[3]&& !avoiding && distance[1] <= breaking_dist)
  {
    spdA = ((double)spdA * breaking_cof * (double)distance[1]);
    spdB = ((double)spdB * breaking_cof * (double)distance[1]);
  }

  //dynamic kP
  if(dips[4])
  {
    
  }

  //constrains motor speeds just in case
  spdA = constrain(spdA, -max_spd, max_spd);
  spdB = constrain(spdB, -max_spd, max_spd);

  //actual driving
  if(spdA==0 && spdB==0)  
  {
    digitalWrite(AIN_1, HIGH);
    digitalWrite(AIN_2, HIGH);
    digitalWrite(BIN_1, HIGH);
    digitalWrite(BIN_2, HIGH);
    
  }  
  else
  {
    //motor A  
    if(spdA > 0)
    {
      analogWrite(AIN_1, spdA);
      digitalWrite(AIN_2, LOW);
    }
    else 
    {
      digitalWrite(AIN_1, LOW);
      analogWrite(AIN_2, -spdA);
    }
    // motor B
    if(spdB > 0)
    {
      analogWrite(BIN_1, spdB);
      digitalWrite(BIN_2, LOW);
    }
    else 
    {
      digitalWrite(BIN_1, LOW);
      analogWrite(BIN_2, -spdB);
    }
  }  
}

int emulate_front() // emulates front sensosr if it's not found
{
  return 0;
}

void avoid()
{
  avoiding = true;
  drive(0, 0);
  delay(100);
  while(distance[1]<= minimal_front_distance)
  {
    sensors();
    drive(-avoid_spd, -avoid_spd);
  }  
  drive(0, 0);
  milk_1 = millis();
  avoiding = false;
  accel = 0;
}


//----------------------- Do not touch this fucking bs ------------------------------------------------
//initialises I2C devices, pins, serial comunication and reads dip switches 
void init_all()
{  

  dipsaliai();

  // pin init
    //vlx
  pinMode(xshut1, OUTPUT); 
  pinMode(xshut2, OUTPUT); 
  pinMode(xshut3, OUTPUT); 
    //motors
  pinMode(AIN_1, OUTPUT); 
  pinMode(AIN_2, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(mot_en, OUTPUT);

  //enables or disables motors
  digitalWrite(mot_en, dips[0]);
  
  // serial init 
  if(dips[1])
    Serial.begin(serial_baud);


  // i2c init
    Wire.begin();


  // vlx init

  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);
  digitalWrite(xshut3, LOW);
  delay(10);

  //------------------------- vlx 1 -----------------------------
  digitalWrite(xshut1, HIGH);
  delay(10);
  VLX_1.setAddress(0x30); // chahnges the default i2c adress
  /*if(vlx_refresh_rate < 20000) // changes vlx timing shit
  {
    VLX_1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12); // minimal values
    VLX_1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8);   
  }  
  else 
  {
    VLX_1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14); // defaults
    VLX_1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  }*/
  VLX_1.setMeasurementTimingBudget(vlx_refresh_rate); // sets refresh rate
  delay(10); // maybe not needed
  if(VLX_1.init()) // checks and initialises the vlx
    vlx[0] = true;
  else
    vlx[0] = false;
  VLX_1.startContinuous(); //starts continuous data reading
  if(dips[1]) // prints adress if serial is enabled
    Serial.println(VLX_1.getAddress());
  
  //------------------------- vlx 2 ------------------------------
  digitalWrite(xshut2, HIGH);
  delay(10); 
  VLX_2.setAddress(0x31);
  /*if(vlx_refresh_rate < 20000)
  {
    VLX_2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12);
    VLX_2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8);    
  }  
  else 
  {
    VLX_2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    VLX_2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  }*/
  VLX_2.setMeasurementTimingBudget(vlx_refresh_rate);
  delay(10);
  if(VLX_2.init())
    vlx[1] = true;
  else
    vlx[1] = false;
  VLX_2.startContinuous();
  if(dips[1])
    Serial.println(VLX_2.getAddress());

  //-------------------- vlx 3 ----------------------------
  digitalWrite(xshut3, HIGH);
  delay(10); 
  VLX_3.setAddress(0x32);
  /*if(vlx_refresh_rate < 20000)
  {
    VLX_3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12);
    VLX_3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8);   
  }  
  else 
  {
    VLX_3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    VLX_3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  }*/
  VLX_3.setMeasurementTimingBudget(vlx_refresh_rate);
  delay(10);
  if(VLX_3.init())
    vlx[2] = true;
  else
    vlx[2] = false;
  VLX_3.startContinuous();
  if(dips[1])
    Serial.println(VLX_3.getAddress());

  //if(dips[1])
    Serial.println("init finishit");


  // gyro 
  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
  }
  Serial.println("Accel Gyro calibration will start in 2sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(2000);
  mpu.calibrateAccelGyro();

  Serial.println("Please Wave device in a figure eight until done.");
  mpu.calibrateMag();

  mpu.verbose(false);  
}

void dipsaliai()//reads dip switches
{

  int d12 = analogRead(A0);
  int d34 = analogRead(A6);
  int d56 = analogRead(A7);

  if( d12 > 1100)
  {
    if( d12 > 1500)
    {
      if( d12 > 3000)
      {
        dips[0] = false;
        dips[1] = false;
      }
      else     
      {
        dips[0] = true;
        dips[1] = false;
      }       
    }
    else     
    {        
      dips[0] = false;
      dips[1] = true;
    }       
  }
  else 
  {
    dips[0] = true;
    dips[1] = true;
  }

  if( d34 > 1100)
  {
    if( d34 > 1500)
    {
      if( d34 > 3000)
      {
        dips[2] = false;
        dips[3] = false;
      }
      else     
      {
        dips[2] = true;
        dips[3] = false;
      }       
    }
    else     
    {        
      dips[2] = false;
      dips[3] = true;
    }       
  }
  else 
  {
    dips[2] = true;
    dips[3] = true;
  }

  if( d56 > 1100)
  {
    if( d56 > 1500)
    {
      if( d56 > 3000)
      {
        dips[4] = false;
        dips[5] = false;
      }
      else     
      {
        dips[4] = true;
        dips[5] = false;
      }       
    }
    else     
    {        
      dips[4] = false;
      dips[5] = true;
    }       
  }
  else 
  {
    dips[4] = true;
    dips[5] = true;
  }

}
