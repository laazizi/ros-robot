#include "WProgram.h"
#include <BatteryMonitor.h>

#define c_ScaledBatteryVInPin 8 // analog input pin for the battery voltage divider
#define c_VInToVBatteryRatio 2.921

BatteryMonitor _BatteryMonitor(c_ScaledBatteryVInPin, c_VInToVBatteryRatio);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  _BatteryMonitor.Update();

  Serial.print(_BatteryMonitor.BatteryVoltage, 4);
  Serial.print("\t");
  Serial.print(_BatteryMonitor.VoltageIsTooLow);
  Serial.print("\n");

  delay(500);
}

