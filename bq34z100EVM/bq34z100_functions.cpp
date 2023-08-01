/*
 * BQ34Z100 library for functions like reading/writing/setting up
 * By Pablo F. Moreno (pablomorenolemus2001@gmail.com)
 * I used some funcionts and based my code by the already 
 * existing library made by Ben V. Brown (https://github.com/Ralim/BQ34Z100/tree/master)
 */


//Authentication key 0x0123456789ABCDEFFEDCBA987654321 is 160 bits

#include <Wire.h>                                                                           // Wire library for communicating over I2C
#include "bq34z100.h" 
extern "C" {
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
}
#include "Arduino.h"

// This is the I2C address of the BQ34Z100
#define BQ34Z100 0x55                         
//GPIO22 I2C_SCL, GPIO21 I2C_SDA, default

//class bq34z100
bq34z100::bq34z100() 
{
  Wire.begin(); //just to ensure i2c starts                                                                         //initiate Wire library for I2C comunication; 
}

/* This function is intende to enable host reading and writing of battery
 * information (for some information, only the registers like voltage an 
 * remaining capacity). Each standar command has an associated command-code pair
 * that consists of two bytes of data, two consecutive I2C transmissions
 * must be executed. Refer to: https://www.ti.com/lit/ds/symlink/bq34z100-g1.pdf?ts=1690033626394
 * in secction 7.3.1.1 for more information.
 * 
 * What is does is that it sends the address of the register that we want to
 * access plus an offset "i" that allows reading multiple consecutive values, after that
 * the transmission ends and we request the 1 byte of data to the device. Then the 
 * obtained byte is shifted left by "(8*i)" bits and added to 'returnVal', This step
 * concaatenates the bytes obtained from multiple read into a single 32-bite value. 
 * In summary it reads 'lenght' bytes of data starting from 'add' (address) specified and 
 * concatenates them into a single variable.
 */
uint32_t bq34z100::Read(int add, uint8_t length)                                            //member of bq34z100
{
    uint32_t returnVal = 0;

    for (int i = 0; i < length; i++)
    {
        Wire.beginTransmission(BQ34Z100);
        Wire.write(add + i);
        Wire.endTransmission();
        Wire.requestFrom(BQ34Z100, 1);
        returnVal = returnVal + (Wire.read() << (8 * i));
    }
    return returnVal;
}

/*
 * this function does the same as read, but this one is useful when the registers are 
 * not a consecutive pair like: 0x01->0x02. 
 */
int bq34z100::readControl(uint8_t add,uint8_t add2)
{
    Wire.beginTransmission(BQ34Z100);
    Wire.write(add2); 
    Wire.write(add);
    Wire.write(0x00); //status of the key features
    Wire.endTransmission();

    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(BQ34Z100, 2);
    int16_t temp = Wire.read();
    temp = temp | (Wire.read() << 8);
    return temp;

}

uint32_t bq34z100:Status()
{
  Wire.write(0x00); //status of the key features 
  Read(0x00,1) 
  for(int i = 0; i < length; i--)
  {
    
  }
}

uint32_t bq34z100::ReadVoltage ()
{
  return Read(0x08, 2); 
  
}

/*
 * This function is intendded to try I2C comunication by finding
 * the address of the chip wich you want to comunicate with. for
 * the bq34z100 you should expect 0x55 if comunication is ok.
 */

uint16_t bq34z100::TryCommunication() 
{
  byte error, address;
  int nDevices;
  return Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      return Serial.print("I2C device found at address 0x");
      if (address<16) {
        return Serial.print("0");
      }
      return Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      return Serial.print("Unknow error at address 0x");
      if (address<16) {
        return Serial.print("0");
      }
      return Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    return Serial.println("No I2C devices found\n");
  }
  else {
    return Serial.println("done\n");
  }
  delay(5000);
}
