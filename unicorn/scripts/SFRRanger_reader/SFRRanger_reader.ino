// I2C SRF10 or SRF08 Devantech Ultrasonic Ranger Finder
// by Nicholas Zambetti <http://www.zambetti.com>
// and James Tichenor <http://www.jamestichenor.net>

// Demonstrates use of the Wire library reading data from the
// Devantech Utrasonic Rangers SFR08 and SFR10

// Created 29 April 2006

// This example code is in the public domain.
// The adresses aer from 0xF0 to 0xF7 for the sensors.

#include <Wire.h>
#include <String.h>

void setup() {
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);          // start serial communication at 9600bps
}


int reading[]= {0,0,0,0,0,0,0,0};      // change the lenght of the array depending on the readings.
String sensorValue =":";
void loop() {
  // step 1: instruct sensor to read echoes
//  for (int i= 0; i <=7; i++)
 // { 
    
    Wire.beginTransmission(0xF0); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting

    Wire.beginTransmission(0xF1); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting

        Wire.beginTransmission(0xF2); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting

    Wire.beginTransmission(0xF3); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting    

    Wire.beginTransmission(0xF4); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting

    Wire.beginTransmission(0xF5); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting
  
        Wire.beginTransmission(0xF6); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting
    
    Wire.beginTransmission(0xF7); // transmit to device #112 (0x70)
    // the address specified in the datasheet is 224 (0xE0)
    // but i2c adressing uses the high 7 bits so it's 112
    Wire.write(byte(0x00));      // sets register pointer to the command register (0x00)
    Wire.write(byte(0x51));      // command sensor to measure in "inches" (0x50)
    // use 0x51 for centimeters
    // use 0x52 for ping microseconds
    Wire.endTransmission();      // stop transmitting

    
    // step 2: wait for readings to happen
    delay(65);                   // datasheet suggests at least 65 milliseconds
  
    // step 3: instruct sensor to return a particular echo reading
    Wire.beginTransmission(0xF0); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting

    Wire.beginTransmission(0xF1); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting

    Wire.beginTransmission(0xF2); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting


    Wire.beginTransmission(0xF3); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting
    
    Wire.beginTransmission(0xF4); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting
    
    Wire.beginTransmission(0xF5); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting    
    
    Wire.beginTransmission(0xF6); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting    
    
    Wire.beginTransmission(0xF7); // transmit to device #112
    Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
    Wire.endTransmission();      // stop transmitting    
    
    
    
    // step 4: request reading from sensor
    Wire.requestFrom(0xF0, 2);    // request 2 bytes from slave device #112
        if (2 <= Wire.available()) { // if two bytes were received
      reading[0] = Wire.read();  // receive high byte (overwrites previous reading)
      reading[0] = reading[0] << 8;    // shift high byte to be high 8 bits
      reading[0] |= Wire.read(); // receive low byte as lower 8 bits
      sensorValue += reading[0];
      sensorValue += ":";
        }
    
    Wire.requestFrom(0xF1, 2);    // request 2 bytes from slave device #112
    if (2 <= Wire.available()) { // if two bytes were received
      reading[1] = Wire.read();  // receive high byte (overwrites previous reading)
      reading[1] = reading[1] << 8;    // shift high byte to be high 8 bits
      reading[1] |= Wire.read(); // receive low byte as lower 8 bits
      sensorValue += reading[1];
      sensorValue += ":";
    }

    
    Wire.requestFrom(0xF2, 2);    // request 2 bytes from slave device #112
    if (2 <= Wire.available()) { // if two bytes were received
      reading[2] = Wire.read();  // receive high byte (overwrites previous reading)
      reading[2] = reading[2] << 8;    // shift high byte to be high 8 bits
      reading[2] |= Wire.read(); // receive low byte as lower 8 bits
      sensorValue += reading[2];
      sensorValue += ":";
    }
      
    Wire.requestFrom(0xF3, 2);    // request 2 bytes from slave device #112
    if (2 <= Wire.available()) { // if two bytes were received
      reading[3] = Wire.read();  // receive high byte (overwrites previous reading)
      reading[3] = reading[3] << 8;    // shift high byte to be high 8 bits
      reading[3] |= Wire.read(); // receive low byte as lower 8 bits
      sensorValue += reading[3];
      sensorValue += ":";
    }
    
    Wire.requestFrom(0xF4, 2);    // request 2 bytes from slave device #112
    if (2 <= Wire.available()) { // if two bytes were received
      reading[4] = Wire.read();  // receive high byte (overwrites previous reading)
      reading[4] = reading[4] << 8;    // shift high byte to be high 8 bits
      reading[4] |= Wire.read(); // receive low byte as lower 8 bits
      sensorValue += reading[4];
      sensorValue += ":";
    }
    
    Wire.requestFrom(0xF5, 2);    // request 2 bytes from slave device #112
    if (2 <= Wire.available()) { // if two bytes were received
      reading[5] = Wire.read();  // receive high byte (overwrites previous reading)
      reading[5] = reading[5] << 8;    // shift high byte to be high 8 bits
      reading[5] |= Wire.read(); // receive low byte as lower 8 bits
      sensorValue += reading[5];
      sensorValue += ":";
    }

    
    Wire.requestFrom(0xF6, 2);    // request 2 bytes from slave device #112                        
        if (2 <= Wire.available()) { // if two bytes were received
      reading[6] = Wire.read();  // receive high byte (overwrites previous reading)
      reading[6] = reading[6] << 8;    // shift high byte to be high 8 bits
      reading[6] |= Wire.read(); // receive low byte as lower 8 bits
      sensorValue += reading[6];
      sensorValue += ":";
        }
    Wire.requestFrom(0xF7, 2);    // request 2 bytes from slave device #112
    if (2 <= Wire.available()) { // if two bytes were received
      reading[7] = Wire.read();  // receive high byte (overwrites previous reading)
      reading[7] = reading[7] << 8;    // shift high byte to be high 8 bits
      reading[7] |= Wire.read(); // receive low byte as lower 8 bits
      sensorValue += reading[7];
      sensorValue += ":";
    }
  
    // step 5: receive reading from sensor

//  }
  Serial.println(sensorValue);
  sensorValue =":";  
}
    


/*

// The following code changes the address of a Devantech Ultrasonic Range Finder (SRF10 or SRF08)
// usage: changeAddress(0x70, 0xE6);

void changeAddress(byte oldAddress, byte newAddress)
{
  Wire.beginTransmission(oldAddress);
  Wire.write(byte(0x00));
  Wire.write(byte(0xA0));
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write(byte(0x00));
  Wire.write(byte(0xAA));
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write(byte(0x00));
  Wire.write(byte(0xA5));
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write(byte(0x00));
  Wire.write(newAddress);
  Wire.endTransmission();
}

*/
