#include <Wire.h>
//#include "SRF02.h"

int currentDeviceId = 0x70;
int New_Address = 0xF7;
/*
 * SÄTT ADDRESSERNA FÖR ULTRA LJUD FRÅN 0xF0 TILL 0xF7 !!!!!!

*/
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  configureDeviceId(currentDeviceId, New_Address);
  Serial.println("setup done");
}

void loop()
{
//Serial.println(New_Address);
}

void configureDeviceId(int currentDevice, int New_Address)
{
 Wire.begin();
 Wire.beginTransmission(currentDeviceId);
 Wire.write(0);
 Wire.write(0xA0);
 Wire.endTransmission();
  
 Wire.beginTransmission(currentDeviceId);
 Wire.write(0);
 Wire.write(0xAA);
 Wire.endTransmission();
 
 Wire.beginTransmission(currentDeviceId);
 Wire.write(0);
 Wire.write(0xA5);
 Wire.endTransmission();
 
 Wire.beginTransmission(currentDeviceId);
 Wire.write(0);
 Wire.write(New_Address << 1);
 Wire.endTransmission();
  Serial.println("done");
  }
