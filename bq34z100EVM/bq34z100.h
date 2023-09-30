#ifndef BQ34Z100_H
#define BQ34Z100_H
#include <inttypes.h>
#include "Arduino.h"
#include "Wire.h"  // I2C library
#include <math.h> 
#include "Convert.h"
 
class bq34z100
{
    
  private:
      //read can only work inside this library and not in the 
      uint16_t Read(int add, uint8_t length); //
      void FULL_ACCESS_MODE();
      void UNSEALED();
      void write_reg(uint8_t add, uint8_t val);
  
  public:
      void begin();
      int writeConfig();//should only be done once
      bq34z100();
      uint16_t DeviceType();
      uint16_t ChemID();
      uint16_t FullChargeCapacity();
      uint8_t getSOC();//gets the current state of charge %
      uint16_t getTemp();//returns the temperature in C
      uint16_t  getVoltage();//returns the battery voltage
      int getCapacity();//returns the current battery capacity
      int16_t getCurrent();//returns the current flowing atm
      int getStatus();//returns the flags
      int getRemaining();
      uint16_t getFlags();
      int16_t getFlagsB();
      int16_t Control(uint8_t, uint8_t);
      int readInstantCurrent();
      bool readFlash(uint8_t , uint8_t );
      void reset();
      int16_t RESET_DATA();
      int enableIT();
      int enableCal();
      int exitCal();
      int enterCal();
      void checkSum(uint16_t , uint8_t);
      uint16_t CalibrateVoltageDivider(uint16_t);//used to calibrate the voltage divider
      int16_t CalculatePackParameters(uint8_t i_pack, uint8_t v_pack, uint8_t Q_cell);
      void writeFlash(uint8_t, uint8_t);
      void chgFlashPair(uint8_t, int);
      void chg104Table(uint16_t Vdivider,float CCGain,float CCDelta);
      void chg48Table(uint16_t, uint16_t);
      void chg49Table(uint16_t, uint16_t, uint16_t, uint16_t);
      void chg64Table(uint16_t, uint16_t, uint8_t, uint8_t);
      void chg83Table(uint16_t);
      void chg68Table(uint16_t);
      void chg80Table(uint16_t);
      void chg82Table(uint16_t, uint16_t);
      void TryCommunication();
      void writeTable(uint16_t , uint8_t );
      uint16_t readVDivider();//reads the currently set voltage divider
      uint32_t floatToXemics(float f);
      float XemicsTofloat(uint32_t inVal);
      int setup(uint8_t BatteryChemistry,uint8_t SeriesCells,uint16_t CellCapacity,uint16_t PackCurrentVoltage,uint16_t current);
      int16_t Status();
      int16_t SERNUM();
      void chgFlashQuad(uint8_t index, uint32_t value);
      void CalibrateCurrentShunt(int16_t current);
      float readCurrentShunt();
      int8_t LearnedStatus();

};
#endif
