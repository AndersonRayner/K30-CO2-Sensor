#include <K30_CO2.h>

#define _PIN_CALIBRATE_CO2_IN 40

K30_CO2 k30(Wire, _PIN_CALIBRATE_CO2_IN);

void setup()
{
  // Set up the comms ports
  Serial.begin(115200);
  Serial.print("K30 CO2 Sensor Test Program\n");
  Serial.print("============================\n");

  Wire.begin();
  
  // Set up the encoder
  k30._debug = 0;
  k30.init();

  k30.calibrate();
  
}

void loop() 
{

  // Update the sensor
  k30.update();

  // Print the encoder pulses
  char buf[50]; sprintf(buf, "CO2: %hu [ ppm ]\n",k30.get_CO2());
  Serial.print(buf);
  delay(2000);
    
}
