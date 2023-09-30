#include "bq34z100.h"
#include "Wire.h"
#include "Convert.h"

bq34z100 BQ34Z100;
Convert Convert;

char CURRENT[8];

void setup()
{
  //Set up will only excecute once
  Serial.begin(115200);
  Wire.begin();
  BQ34Z100.chg48Table(2000, 7200); //2000 mAh y 7200 mWh
  BQ34Z100.reset();
  //pinMode(13, OUTPUT);
  //digitalWrite(13, LOW);
  //Auto calibration (Run this once on the bq34z100 to calibrate most of the internal registers
  //Battery Chemistry
  //Number of series Batteries
  //Battery capacity in mAh
  //Current voltage on the pack in mV (eg 12.369V)
  // Current being applied to the pack for currentShunt Cal in mA (must be > 200mA)
  //BQ34Z100.setup(0x101,3,8000,12369,1000);

  delay(50);
    
}


void loop()
{
  //Serial.println("Voltage en mV");
  //dtostrf(BQ34Z100.getVoltage(),6,1,VOLTAGE);
  //Serial.println(VOLTAGE);

  Serial.print(BQ34Z100.getVoltage());
  Serial.println("mV");

  Serial.print(BQ34Z100.getCurrent());
  Serial.println("mA");

  Serial.print(BQ34Z100.getTemp());
  Serial.println("°C");

  Serial.print(BQ34Z100.getSOC());
  Serial.println("%");

  //BQ34Z100.TryCommunication();
  //Serial.println("Gas Gauge Status register (FLags)");
  //Serial.println(BQ34Z100.getFlags(), BIN);
   
  //Serial.println("Gas Gauge Control Status register");
  //Serial.println(BQ34Z100.Status(), BIN);  

  //Serial.println("Device type");
  //Serial.println(BQ34Z100.DeviceType(), HEX);  

  //Serial.println("Chemical ID");
  //Serial.println(BQ34Z100.ChemID(), HEX);  
  delay(2000);//delay 2 seconds 

}
