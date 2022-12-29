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
//#include "MPU9250.h"


// pinout
#define XSHUT1 4
#define XSHUT2 7
#define XSHUT3 8
#define AIN_1 9
#define AIN_2 10
#define BIN_1 5
#define BIN_2 6
#define MOT_EN 11
#define DIP_1 A0
#define DIP_2 19
#define DIP_3 20


// operation modes
#define DEBUGGG 

// initialisation things 
#define SERIAL_BAUD 115200
#define VLX_REFRESH_RATE 20000//default 20000us (20ms) may be useful for overclocking

// devices
VL53L0X VLX_1;
VL53L0X VLX_2;
VL53L0X VLX_3; 
//MPU9250 mpu;


// constants
#define START_DELAY 5000

#define BREAKING_COF 0.0025
#define ACCEL_COF 0.8 

#define MAX_SPD 255
#define MIN_SPD 70
#define AVOID_SPD 180
#define DEFAULT_DRIVE_SPD 150

#define MAX_SENSOR_RANGE 1200
#define MIN_SENSOR_RANGE 60

#define MINIMAL_FRONT_DISTANCE 150
#define AVOID_DIST 65
#define BREAKING_DIST 300
#define REACCEL_DIST 80
#define DYNAMIC_KP_COF 0.05


// variables
bool dips[6];
bool vlx[3];
int distance[3];
//int gyro[3];//z, x, y
//int accelg[3];//x, y, z
//int spds[2];
int speed; // front speed in m/s
unsigned long milk_0; // for start delay and measurements 
unsigned long milk_1; // for acceleration
bool avoiding = false;
bool accelerating=false;
bool breaking = false;
int accel = 0;
double PID;


// PID
#define DEFAULT_KP 0.6
#define DEFAULT_KI 0
#define DEFAULT_KD 0.1
double kP = 1.5;



int lastP =0;
int I=0;
int P;
#define target 0
int position; 



// functions
void init_all(); //also reading the bs from dips
void sensors();
void drive(int spdA, int spdB);
void not_moving();


void setup() 
{
  // put your setup code here, to run once
  milk_0 = millis();
  init_all();
  if(dips[5])// does the delay if dip 6 on
  {
    while(millis()-milk_0 <= START_DELAY)
    {
      digitalWrite(LED_BUILTIN, LOW);
    }    
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() 
{
  sensors();
  //digitalWrite(LED_BUILTIN, LOW);
  P = distance[2]-distance[0];
  //P = target - position;
  I += P;
  PID = ((double)P*kP) + ((double)I*DEFAULT_KI) + ((double)(P-lastP)*DEFAULT_KD);
  drive(DEFAULT_DRIVE_SPD+PID, DEFAULT_DRIVE_SPD-PID);
  if(dips[1])
  {
    //Serial.println(PID);
  }
  //digitalWrite(LED_BUILTIN, HIGH);
}

void sensors()
{
  distance[0]=0;
  if(vlx[0])
    distance[0]=VLX_1.readRangeContinuousMillimeters();//vlx1 left    
    
  if(distance[0] <=MIN_SENSOR_RANGE)
    distance[0]=1;    

  if(distance[0] <= REACCEL_DIST && !accelerating)
    milk_1 = millis();
        
  distance[2]=0;
  if(vlx[2])
    distance[2]=VLX_3.readRangeContinuousMillimeters();// vlx3 right
   
  if(distance[2] <=MIN_SENSOR_RANGE)
    distance[2]=1;    

  if(distance[2] <= REACCEL_DIST && !accelerating)
    milk_1 = millis();   


  if(vlx[1])
    distance[1]=VLX_2.readRangeContinuousMillimeters();//vlx2 mid
  else 
    distance[1]=emulate_front();

  if(distance[1] <=MIN_SENSOR_RANGE)
    distance[1]=1; 

  if(distance[1] <= AVOID_DIST && !avoiding)
    avoid(); 
  
  

  if(dips[1])
  {
    //Serial.print("1: ");
    Serial.print(distance[0]);
    Serial.print(" ");
    Serial.print(distance[1]);
    Serial.print(" ");
    Serial.println(distance[2]);
  }
}

void drive(int spdA, int spdB)
{

  

  //accel
  if(dips[2] && !avoiding)// uses milk_1
  {
    accelerating = true;
    accel = (millis()-milk_1)*ACCEL_COF;
    accel = constrain(accel, 0, 100);
    if(accel == 100)
      accelerating = false;
    spdA = spdA*accel/100;
    spdB = spdB*accel/100;
  }  

  //braking
  if(dips[3]&& !avoiding && distance[1] <= BREAKING_DIST)
  {
    spdA = ((double)spdA * BREAKING_COF * (double)distance[1]);
    spdB = ((double)spdB * BREAKING_COF * (double)distance[1]);
  }

  //dynamic kP
  kP = DEFAULT_KP;
  if(dips[4] && !avoiding && distance[1] <= BREAKING_DIST)
  {
    kP = DEFAULT_KP + (BREAKING_DIST-distance[1]) * DYNAMIC_KP_COF;
  }

  //constrains motor speeds just in case
  spdA = constrain(spdA, -MAX_SPD, MAX_SPD);
  spdB = constrain(spdB, -MAX_SPD, MAX_SPD);

  if(0)
  {
    if(spdA == spdB)
    {
      Serial.print("<----( ");
      Serial.print(spdA);
      Serial.println(" )---->");
    }
    else
    {
      if(spdA > spdB)
      {
        Serial.print("---( ");
        Serial.print(spdA);
        Serial.print(" )--( ");
        Serial.print(spdB);
        Serial.println(" )-->");
      }
      else
      {
        Serial.print("<--( ");
        Serial.print(spdA);
        Serial.print(" )--( ");
        Serial.print(spdB);
        Serial.println(" )---");
      }
    }
  }

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
  return 666;
}

void avoid()
{
  avoiding = true;

  if(distance[0] >= distance[2])
  {
    while(distance[1]<= AVOID_DIST+100)
    {
      sensors();
      if(dips[1])
        Serial.println("Avoiding");
      drive(-AVOID_SPD, -0.5*AVOID_SPD);
    }  
  }
  else
  {
    while(distance[1]<= AVOID_DIST+100)
    {
      sensors();
      if(dips[1])
        Serial.println("Avoiding");
      drive(-0.5*AVOID_SPD, -AVOID_SPD);
    } 
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
  pinMode(XSHUT1, OUTPUT); 
  pinMode(XSHUT2, OUTPUT); 
  pinMode(XSHUT3, OUTPUT); 
    //motors
  pinMode(AIN_1, OUTPUT); 
  pinMode(AIN_2, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(MOT_EN, OUTPUT);

  //enables or disables motors
  digitalWrite(MOT_EN, dips[0]);
  
  // serial init 
  if(dips[1])
    Serial.begin(SERIAL_BAUD);


  // i2c init
  Wire.begin();


  // vlx init

  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(10);

  //------------------------- vlx 1 -----------------------------
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  VLX_1.setAddress(0x30); // chahnges the default i2c adress
  /*if(VLX_REFRESH_RATE < 20000) // changes vlx timing shit
  {
    VLX_1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12); // minimal values
    VLX_1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8);   
  }  
  else 
  {
    VLX_1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14); // defaults
    VLX_1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  }*/
  VLX_1.setMeasurementTimingBudget(VLX_REFRESH_RATE); // sets refresh rate
  delay(10); // maybe not needed
  if(VLX_1.init()) // checks and initialises the vlx
    vlx[0] = true;
  else
    vlx[0] = false;
  VLX_1.startContinuous(); //starts continuous data reading
  if(dips[1]) // prints adress if serial is enabled
    Serial.println(VLX_1.getAddress());
  
  //------------------------- vlx 2 ------------------------------
  digitalWrite(XSHUT2, HIGH);
  delay(10); 
  VLX_2.setAddress(0x31);
  /*if(VLX_REFRESH_RATE < 20000)
  {
    VLX_2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12);
    VLX_2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8);    
  }  
  else 
  {
    VLX_2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    VLX_2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  }*/
  VLX_2.setMeasurementTimingBudget(VLX_REFRESH_RATE+5000);
  delay(10);
  if(VLX_2.init())
    vlx[1] = true;
  else
    vlx[1] = false;
  VLX_2.startContinuous();
  if(dips[1])
    Serial.println(VLX_2.getAddress());

  //-------------------- vlx 3 ----------------------------
  digitalWrite(XSHUT3, HIGH);
  delay(10); 
  VLX_3.setAddress(0x32);
  /*if(VLX_REFRESH_RATE < 20000)
  {
    VLX_3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12);
    VLX_3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8);   
  }  
  else 
  {
    VLX_3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    VLX_3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  }*/
  VLX_3.setMeasurementTimingBudget(VLX_REFRESH_RATE);
  delay(10);
  if(VLX_3.init())
    vlx[2] = true;
  else
    vlx[2] = false;
  VLX_3.startContinuous();
  if(dips[1])
    Serial.println(VLX_3.getAddress());

  if(dips[1])
    Serial.println("init finishit");

  
}

void dipsaliai()//reads dip switches
{

    int d12 = analogRead(DIP_1);
    int d34 = analogRead(DIP_2);
    int d56 = analogRead(DIP_3);

    dips[0] = true;;
    dips[1] = true;
    if( d12 > 1100)
    {
        dips[0] = false;
        dips[1] = true;
        if( d12 > 1500)
        {
            dips[0] = true;
            dips[1] = false;
            if( d12 > 3000)
            {
                dips[0] = false;
                dips[1] = false;
            }     
        }      
    }
    dips[2] = true;
    dips[3] = true;
    if( d34 > 1100)
    {
        dips[2] = false;
        dips[3] = true;
        if( d34 > 1500)
        {
            dips[2] = true;
            dips[3] = false;
            if( d34 > 3000)
            {
                dips[2] = false;
                dips[3] = false;
            }     
        }      
    }

    dips[4] = true;
    dips[5] = true;
    if( d56 > 1100)
    {
        dips[4] = false;
        dips[5] = true;
        if( d56 > 1500)
        {
            dips[4] = true;
            dips[5] = false;     
            if( d56 > 3000)
            {
                dips[4] = false;
                dips[5] = false;
            }   
        }
    }
}