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

// This is the I2C address of the BQ34Z100 0x55 or 0xD5
#define BQ34Z100 0x55                         
//GPIO22 I2C_SCL, GPIO21 I2C_SDA, default

//class bq34z100
bq34z100::bq34z100() 
{
  Wire.begin(); //just to ensure i2c starts, initiate Wire library for I2C comunication; 
}


/*----------------------------------INTERFACE------------------------------------------------------
 *  
 * range estimation. communications, data recording, reporting.
 * 
   * bq34z100::Read is intende to enable host reading and writing of battery
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
   * concatenates the bytes obtained from multiple read into a single 32-bite value. 
   * In summary it reads 'lenght' bytes of data starting from 'add' (address) specified and 
   * concatenates them into a single variable.
 */
uint16_t bq34z100::Read(int add, uint8_t length)  //member of bq34z100  
{
    uint16_t returnVal = 0; //revisar si se puede de 16 bits  

    for (int i = 0; i <= length; i++)
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
  * 'bq34z100::readControl' does the same as 'bq34z100::read', but this one is useful when the registers are 
  * not a consecutive pair like: 0x01->0x02. it's main porpuse is to read the control registers (2.1.1 from 
  * the bq34z100G1 technical refernece manual).
  * 
  */

int16_t bq34z100::Control(uint8_t subcommand)
{
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(subcommand); 
    Wire.endTransmission();

    Wire.beginTransmission(BQ34Z100);   
    Wire.write(0x00);   
    Wire.endTransmission();
    Wire.requestFrom(BQ34Z100, 2);
    int16_t temp = Wire.read();
    temp = temp | (Wire.read() << 8);
    return temp;

}

int bq34z100::enableIT()
{
  return Control(0x21);
}

int16_t bq34z100::Status()
{
  return Control(0x00);
}

int8_t bq34z100::DeviceType()
{
  return Control(0x01);
}

  /*
  * 'bq34z100::TryCommunication' is intendded to try I2C comunication 
  * by finding the address of the chip wich you want to comunicate with. 
  * for the bq34z100 you should expect 0x55 if comunication is ok.
  */

void bq34z100::TryCommunication()
{
  byte error, address;
  int devicesFound;
  devicesFound = 0;
  Serial.println("Scanning...");

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      devicesFound++;

    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (devicesFound == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);
}

  //---------------------------------------------STANDAR DATA COMMANDS


//--------------------------------------------END OF INTERFACE------------------------------------------------------



/*---------------------------------------SENSING AND HIGH-VOLTAGE CONTROL-------------------------------------------
 *            
 * measure voltage, current, temperature; control contactor, 
 * pre-charge; grund.fault detection, thermal managment
 *
 */

int16_t CalculatePackParameters(uint8_t i_cell, uint8_t v_cell, uint8_t Q_cell, uint8_t i_pack, uint8_t v_pack)
{
  uint8_t i_packC = 0; //current that it's going to be calculated
  uint8_t v_packC = 0; //voltage that it's going to be calculated
  uint8_t np = 0; //number of cells in parallel 
  uint8_t ns = 0; //number of cells in series
  uint8_t Q_pack = 0; //Capacity of the pack (Ah)
  uint8_t E_pack = 0; //Total pack energy (w)
  uint8_t P_pack = 0; //Total pack power (J)

  while (i_pack < i_packC)
  {
    int16_t np = 0;
    i_packC = np*i_cell; 
    np++;
    return np;
  }
}


float bq34z100::getVoltage()
{
  uint16_t rawData = Read(0x08, 1); //gets the voltage in binary type
  // Maximum raw value
  const uint16_t maxRawValue = 65535; //the maximun voltage that can be readed is 65535 mV

  // Maximum voltage in millivolts
  const float maxVoltage = 65535.0;

  // Convert the raw data to a floating-point voltage
  float voltage = (rawData / maxRawValue) * maxVoltage;

  return rawData;
}

float bq34z100::getCurrent()
{
  int16_t rawData = Read(0x0A, 1); //gets the current in binary type, every bit is 2mA

}

float bq34z100::getTemp()
{
  uint16_t rawData = Read(0x0C, 1); //gets the temperature in K
}
/*-----------------------------------END OF SENSING AND HIGH-VOLTAGE CONTROL-----------------------------------------



/*-------------------------------------------------------PROTECTION-------------------------------------------------
 *                             
 * against: Over-charge, over-discharge, over-current, short circuit, 
 * extreme temperatures
 * 
 */


//------------------------------------------------END OF PROTECTION-------------------------------------------------



/*---------------------------------------------------PERFORMNCE MANAGMENT-------------------------------------------
 *                 
 * State of charge (SOC) estimation, power-limit computation, balance/equalize cells
 * 
 */


//-----------------------------------------------END OF PERFORMNCE MANAGMENT----------------------------------------



/*------------------------------------------------------DIAGNOSCTIC--------------------------------------------------
 *                 
 * Abuse detection, sate of health (SOH) estimation, state of life (SOL) estimation.
 * 
 */


//-----------------------------------------------------END OF DIAGNOSCTIC---------------------------------------------
