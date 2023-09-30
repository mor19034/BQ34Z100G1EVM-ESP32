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
#include "Convert.h"
}
#include "Arduino.h"

// This is the I2C address of the BQ34Z100 0x55 or 0xD5
#define BQ34Z100 0x55                       
//GPIO22 I2C_SCL, GPIO21 I2C_SDA, default

uint8_t flashbytes[32] = {0}; 

//class bq34z100
bq34z100::bq34z100() 
{
  Wire.begin(); //just to ensure i2c starts, initiate Wire library for I2C comunication; 
}


/*----------------------------------INTERFACE------------------------------------------------------
 *  
 * range estimation. communications, data recording, reporting and calibration
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
  *to write first goes control (0x00) then the command in reverse, for example the command for chem id is 0x0008, so the order to access
  *that is 0x00 for subcommand1 and 0x08 for subcommand2 
  */

int16_t bq34z100::Control(uint8_t subcommand1, uint8_t subcommand2) 
{
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x00);
    Wire.write(subcommand2);
    Wire.write(subcommand1); 
    Wire.endTransmission();

    Wire.beginTransmission(BQ34Z100);   
    Wire.write(0x00);   
    Wire.endTransmission();
    Wire.requestFrom(BQ34Z100, 2);
    int16_t temp = Wire.read();
    temp = temp | (Wire.read() << 8);
    return temp; //return data in decimal 

}

  /*
  * bq34z100::TryCommunication' is intendded to try I2C comunication 
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

  //-------------------------------------STANDAR DATA COMMANDS

int bq34z100::enableIT()
{
  return Control(0x00, 0x21);
}

//
int16_t bq34z100::Status()
{
  int16_t temp = Control(0x00, 0x00);
  byte FlagS= B00000000;
  delay(100);
/*
  if (temp<<15 == 1){
    Serial.print("BQ34Z100-G1 is in FULL ACCESS SEALED");
    FlagS = B00000011;
    if(temp<<15 != 1){
      Serial.print("BQ34Z100-G1 is not in FULL ACCESS SEALED");
      FlagS = B00000100;
    }
  }

  if (temp<<14 == 1){
    Serial.print("BQ34Z100-G1 is in SEALED STATE");
    FlagS = B00000001;
    if(temp<<14 != 1){
      Serial.print("BQ34Z100-G1 is in UNSEALED STATE");
      FlagS = B00000010;
    }
  }
  */
  return temp;
}

uint16_t bq34z100::DeviceType()
{
  return Control(0x00, 0x01);
}

uint16_t bq34z100::ChemID()
{
  return Control(0x00, 0x08);
}

void bq34z100::reset()
{
  Control(0x00, 0x41);
}

int16_t bq34z100::RESET_DATA()
{
  Control(0x00, 0x05);
}

  /*
  * Unsealing the device enables full data flash accesing, necessary for pack configuration and more 
  * Refer to section 7.3.3.1 for more reference
  */


void bq34z100::UNSEALED() //when changing to this state, sequence needs to be reapeted 3 times, acording to Texas Instruments.
{
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x00); //Indicates that control is going to be acceced 
    Wire.write(0x14); //First 2 byte key autentication 
    Wire.write(0x04);
    Wire.endTransmission();

    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x00);
    Wire.write(0x72); 
    Wire.write(0x36); 
    Wire.endTransmission();

    delay(100); //the datasheet indicates to leave a 100 ms delay to ensure modifications of the SEALED state
}

void bq34z100::FULL_ACCESS_MODE()
{
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0xFF);
    Wire.write(0xFF);
    Wire.write(0xFF);
    Wire.write(0xFF); 
    Wire.endTransmission();
    delay(120);
}

void bq34z100::write_reg(uint8_t add, uint8_t val) //the purpose of this function is only to shorten the writing of 2 byte pairs 
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(add); //set up command 
  Wire.write(val); 
  Wire.endTransmission();
}
  //-------------------------------------CALIBRATION & CARACTERIZATION COMMANDS

  //This function uses a series of writes that the datasheet says that need to be doone so the dataflash can be acceced
  //the chapter in wich this steps are is: 7.3.3.1 Accessing Data Flash. 

bool bq34z100::readFlash(uint8_t subclass, uint8_t offset) //remember that it needs to be in unsealed mode
{
  write_reg(0x61, 0x00); //0x61 is set up command & 0x00 indicates that flash is going to be acceded
  write_reg(0x3E, subclass); //this sets that the data flash CLASS is going to be accessed. for subclas> All the classes are in table 7-11 of the datasheet
  write_reg(0x3F, offset%32); //0x3F sets that the offset is going to be set, and the offset indicates wich one is going to be used

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40); //request all dataflash registers 
  Wire.endTransmission();
 
  Wire.requestFrom(BQ34Z100, 32);
  for (int i = 0; i < 32; i++)  //the for is here to read all the dataflash 
    {
      flashbytes[i] = Wire.read(); //is intended to be the old check sum value 
    }
  Wire.endTransmission();
  delay(30);
}

void bq34z100::writeFlash(uint8_t subclass, uint8_t offset) //remember that it needs to be in unsealed mode
{
  write_reg(0x61, 0x00); //0x61 is set up command & 0x00 indicates that flash is going to be acceded
  write_reg(0x3E, subclass); //this sets that the data flash CLASS is going to be accessed. for subclas> All the classes are in table 7-11 of the datasheet
  write_reg(0x3F, offset%32); //This sets that the offset is going to be set, and the offset indicates wich one is going to be used

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40);
  Wire.endTransmission();
 
  Wire.requestFrom(BQ34Z100, 32);
  for (int i = 0; i < 32; i++)  //the for is here to read all the dataflash 
    {
      Wire.write(flashbytes[i]); //is intended to ve the old check sum value 
    }
  Wire.endTransmission();
  delay(30);
}

void bq34z100::checkSum(uint16_t subclass, uint8_t offset)
{
  //To actuatually modify the data flash the checksum block needs to be correct, and it changes every time one bit of the flash is changes 
  int chkSum = 0;
  for (int i = 0; i < 32; i++)
  {
    chkSum += flashbytes[i];
  }
  int chkSumTemp = chkSum / 256;
  chkSum = chkSum - (chkSumTemp * 256);
  chkSum = 255 - chkSum;

  write_reg(0x60, chkSum); //0x60 says that the check sum is going to be checked 
  delay(200); 
}

void bq34z100::chgFlashPair(uint8_t index, int value)
{
    if (index > 31)
        index = index % 32;

    //  change flashbyte first
    flashbytes[index] = value >> 8; //high byte
    flashbytes[index + 1] = value & 0xFF; //lower byte
    // write flashbyte
    write_reg((0x40 + index), flashbytes[index]);
    write_reg((0x40 + index + 1), flashbytes[index + 1]);
}

void bq34z100::chg48Table(uint16_t Design_capacity, uint16_t Design_energy)
{
  readFlash(48, 24);
  chgFlashPair(21, Design_capacity);
  chgFlashPair(23, Design_energy);
  checkSum(48,21);
}
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


uint16_t  bq34z100::getVoltage()
{
  uint16_t rawData = Read(0x08, 1); //gets the voltage in UNSIGNED INTEGER type, the maximun voltage that can be readed is 65535 mV
  // Maximum raw value
  const uint16_t maxRawValue = 65535; //the maximun voltage that can be readed is 65535 mV
  // Maximum voltage in millivolts
  const float maxVoltage = 65535.0;
  // Convert the raw data to a floating-point voltage
  float voltage = (static_cast<float>(rawData) / maxRawValue) * maxVoltage;

  return voltage;
}

int16_t bq34z100::getCurrent()
{
  int16_t rawData = Read(0x0A, 1); //gets the current in binary type, every bit is 1mA
  return rawData;
}

uint16_t bq34z100::getTemp()
{
  uint16_t rawData = Read(0x0C, 1); //gets the temperature in a range of 0 to 6553.5 K
  // Maximum raw value
  const uint16_t maxRawValue = 65535; 
  // Maximum Temperature in Kelvin
  const float maxTemperature = 6553.5;
  // Convert the raw data to a floating-point voltage
  float Temperature = ((static_cast<float>(rawData) / maxRawValue) * maxTemperature) -273.0; //Temperature in °C

  return Temperature;
}

uint8_t bq34z100::getSOC()
{
    return Read(0x02, 0) ; 
}


/*-----------------------------------END OF SENSING AND HIGH-VOLTAGE CONTROL-----------------------------------------



/*-------------------------------------------------------PROTECTION-------------------------------------------------
 *                             
 * against: Over-charge, over-discharge, over-current, short circuit, 
 * extreme temperatures
 * 
 */

uint16_t bq34z100::getFlags()
{
  uint16_t rawData = Read(0x0E, 1); 
  return rawData;
}

  //This read-word function returns the contents of the gas-gauge status register, depicting current operation status. 
  //refer to section 7.3.1.11 of the BQ34Z100-G1 datasheet
int16_t bq34z100::getFlagsB()
{
  return Read(0x12, 1);
}

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
