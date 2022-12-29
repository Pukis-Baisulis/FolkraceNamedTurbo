#include <MPU9250.h>
#include <Wire.h>

MPU9250 mpu;



void setup()
{
    Wire1.setSCL(5);
    Wire1.setSDA(4);
    Wire1.begin();
}

void loop()
{

}