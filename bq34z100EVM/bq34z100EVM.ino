#include "bq34z100.h"
#include "Wire.h"
bq34z100 BQ34Z100;

void setup()
{
  //Set up will only excecute once
  Serial.begin(115200);
  Wire.begin();
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  //Auto calibration (Run this once on the bq34z100 to calibrate most of the internal registers
  //Battery Chemistry
  //Number of series Batteries
  //Battery capacity in mAh
  //Current voltage on the pack in mV (eg 12.369V)
  // Current being applied to the pack for currentShunt Cal in mA (must be > 200mA)
  BQ34Z100.setup(0x101,3,8000,12369,1000);

  delay(200);
    
}


void loop()
{
  digitalWrite(13, HIGH);
  Serial.println(BQ34Z100.getVoltage(), DEC);
  digitalWrite(13, LOW);
  delay(2000);//delay 2 seconds 
}
