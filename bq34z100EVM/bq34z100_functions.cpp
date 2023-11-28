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
uint8_t Old_Check_Sum[1] = {0};
uint8_t Old_byte_value[1] = {0};
byte new_Pack_Configuration_MSB = 0;

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

int16_t bq34z100::Read_Control(uint8_t subcommand1, uint8_t subcommand2) 
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

void bq34z100::write_reg(uint8_t command, uint8_t val) //the purpose of this function is only to shorten the writing of 2 byte pairs 
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(command); //set up command 
  Wire.write(val); 
  Wire.endTransmission();
}


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

  //-------------------------------------STANDAR DATA COMMANDS

int bq34z100::enableIT()
{
  return Read_Control(0x00, 0x21);
}

//
int16_t bq34z100::Status()
{
  int16_t temp = Read_Control(0x00, 0x00);
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
  return Read_Control(0x00, 0x01);
}

uint16_t bq34z100::ChemID()
{
  return Read_Control(0x00, 0x08);
}

void bq34z100::reset()
{
  Read_Control(0x00, 0x41);
}

int16_t bq34z100::RESET_DATA()
{
  Read_Control(0x00, 0x05);
}

  /*
  * Unsealing the device enables full data flash accesing, necessary for pack configuration and more 
  * Refer to section 7.3.3.1 for more reference
  */

void bq34z100::UNSEAL() //when changing to this state, sequence needs to be reapeted 3 times, acording to Texas Instruments.
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

  //-------------------------------------CALIBRATION & CARACTERIZATION COMMANDS

  /*
    The voltage divider 
  */
uint16_t bq34z100::CalibrateVoltageDivider(uint16_t currentVoltage)
{
    if(currentVoltage<5000)
        return 0;
//So do this we set the voltage divider to 1.5 Times the current pack voltage (to start with)
 float setVoltage =   ((float)readVDivider());
 //chg104Table((uint16_t)setVoltage);//set voltage divider
 float readVoltage = (float)getVoltage();
float newSetting = (currentVoltage/readVoltage)*setVoltage;
chg104Table((uint16_t)newSetting,0,0);//set new divider ratio
return (uint16_t)newSetting;
}

void bq34z100::CalibrateCurrentShunt(int16_t current)
{
    if(current>-200 && current<200)
        return;//too small to use to calibrate
    //current is in milliamps
    if(current<0)
        current=-current;
    int16_t currentReading = getCurrent();
    if(currentReading<0)
        currentReading = -currentReading;
    if(currentReading==0)
        currentReading=20;
    Serial.println(currentReading);
    readFlash(0x68, 15);
    delay(30);
  
    uint32_t curentGain = ((uint32_t)flashbytes[0])<<24 | ((uint32_t)flashbytes[1])<<16|((uint32_t)flashbytes[2])<<8|(uint32_t)flashbytes[3];
    Serial.println(curentGain,DEC);
    float currentGainResistance = (4.768/XemicsTofloat(curentGain));
    Serial.println(currentGainResistance);
    float newGain = (((float)currentReading)/((float)current)) * currentGainResistance;
    Serial.println(newGain);
    //we now have the new resistance calculated
Serial.println("--");
    chg104Table(0,newGain,newGain);
    //chg104Table(0,5,5);

    delay(30);
}

//--------------------------------------------DATA FLASH MODIFICATION COMMANDS------------------------------------------------------

void bq34z100::chg48Table(uint16_t designCap, uint16_t designEnergy, uint16_t CellVT1T2, uint16_t CellVT2T3, uint16_t CellVT3T4, uint16_t CellVT4T5)
{
    readFlash(48, 24);
    chgFlashPair(21, designCap);
    chgFlashPair(23, designEnergy);
    chgFlashPair(28, CellVT1T2);//0x0E10 = 3600
    chgFlashPair(30, CellVT2T3);//0x0E10 = 3600
    checkSum(48, 24);
    delay(300);
    readFlash(48, 35);
    chgFlashPair(32, CellVT3T4);//0x0E10 = 3600
    chgFlashPair(34, CellVT4T5);//0x0E10 = 3600
    checkSum(48, 35);
}

void bq34z100::chg40Table()
{
  UNSEAL(); //The secuence repeats 3 times to ensuere unsealing (acording to TI)
  UNSEAL();
  UNSEAL();
  delay(30);
  FULL_ACCESS_MODE();

  new_Pack_Configuration_MSB |= B00000100; 

  write_reg(0x61, 0x00);
  write_reg(0x3E, 0x40);
  write_reg(0x3F, 0x00);

  //This part is for storing the old pack configuration 
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40); //Read the old Pack Configuration
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100, 1);
  Old_byte_value[1] = Wire.read(); //store old Pack Configuration
  Wire.endTransmission();
  delay(30);

  //This part is for storing the old pack CheckSum byte
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x60); //request all dataflash registers 
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100, 1);
  Old_Check_Sum[1] = Wire.read(); //is intended to be the old check sum value 
  Wire.endTransmission();
  delay(30);
  
  //Here we write the new Pack Configuration, where we change the third bite of the MSB
  write_reg(0x4B, new_Pack_Configuration_MSB);

  int temp = (255-Old_Check_Sum[1]-Old_byte_value[1])%256;
  int New_checksum = 255 - (temp + new_Pack_Configuration_MSB)%256;

  delay(30);
  //Reset the gauge to ensure the new data flash parameter goes into effect
  reset();
}

//If vDiv is <500 dont change vdiv
//if CCG && CCD ==0 then dont change them
void bq34z100::chg104Table(uint16_t Vdivider,float CCGain,float CCDelta)
{
    if(Vdivider>32768)
        return;
    
        
    readFlash(0x68, 15);
    delay(30);
    if(Vdivider>500)
    chgFlashPair(14, Vdivider);
    if(!(CCGain==0 && CCDelta==0))
    {
        float GainDF = 4.768/CCGain;
        float DeltaDF = 5677445/CCDelta;
        chgFlashQuad(0,floatToXemics(GainDF));
        chgFlashQuad(4,floatToXemics(DeltaDF));
    }
    checkSum(0x68, 15);

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


uint16_t bq34z100::readVDivider()
{
readFlash(0x68, 15);
    uint16_t val = (((uint16_t)flashbytes[14]) <<8) | flashbytes[15];
    return val;
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

uint8_t bq34z100::getSOC()
{
    return Read(0x02, 0) ; 
}


//-----------------------------------------------END OF PERFORMNCE MANAGMENT----------------------------------------



/*------------------------------------------------------DIAGNOSCTIC--------------------------------------------------
 *                 
 * Abuse detection, sate of health (SOH) estimation, state of life (SOL) estimation.
 * 
 */


//-----------------------------------------------------END OF DIAGNOSCTIC---------------------------------------------
