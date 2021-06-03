#include "K30_CO2.h"

K30_CO2::K30_CO2(TwoWire& K30_I2C, int CS_CALIB) :
  _K30_I2C(K30_I2C),
  _CS_CALIB(CS_CALIB)
{
    
}

// Functions for the CO2 sensor
void K30_CO2::init()
{
  // Initialise the CO2 sensor pins
  pinMode(_CS_CALIB, OUTPUT);

  // Run the CO2 sensor in normal mode
  digitalWrite(_CS_CALIB, HIGH);

  // Debugging
  if (_debug) { SERIAL_DEBUG.print("K30_CO2: Initialised\n"); }
  
  
  // All done
  _enabled = 1;

  return;
}

void K30_CO2::calibrate()
{
    // Debugging
    if (_debug) { SERIAL_DEBUG.print("K30_CO2: Calibrating..."); }

    unsigned long t_start; 
    
    // Pull DIN1 low to start the calibration and wait
    digitalWrite(_CS_CALIB, LOW);
    t_start = millis();

    do
    {
        // wait for the calibration time to expire
    } while (millis() < t_start + _t_CO2_calibrate);

    digitalWrite(_CS_CALIB, HIGH);

    // Debugging
    if (_debug) { SERIAL_DEBUG.print("Done!\n"); }

    // Sensor calibrated
    return;
}

unsigned long K30_CO2::t_calibrate()
{
    return (_t_CO2_calibrate);
}

void K30_CO2::update()
{
  if (_debug) { SERIAL_DEBUG.print("K30_CO2: Taking a sample\n"); }
  if (_enabled == 0) { return; }
  
  // Takes a sample from the CO2 reader
  K30_CO2_rx_data K30_CO2_data;

  // Request a reading
  _K30_I2C.beginTransmission(_K30_ADDR);
  
  _K30_I2C.write(0x22);
  _K30_I2C.write(0x00);
  _K30_I2C.write(0x08);
  _K30_I2C.write(0x2A);

  _K30_I2C.endTransmission();

  // Wait for the sensor to catch up
  delay(10);

  // Read Reply
  _K30_I2C.requestFrom(_K30_ADDR, K30_CO2_RX_STREAM_BUFFER_SIZE);
  
  if (_debug) { SERIAL_DEBUG.print("CO2: "); SERIAL_DEBUG.print(_K30_I2C.available()); SERIAL_DEBUG.print(" bytes available\n"); }

  if (_K30_I2C.available() >= K30_CO2_RX_STREAM_BUFFER_SIZE)
  {
    while (_K30_I2C.available() > K30_CO2_RX_STREAM_BUFFER_SIZE)
    {
      // Read any old data and clear the buffer down to the last 4 bytes
      if (_debug) { SERIAL_DEBUG.print("CO2: Extra data on bus that wasn't expected\n"); }
      _K30_I2C.read();
      
    }

    // Read the last data received
    for (uint8_t ii = 0; ii < K30_CO2_RX_STREAM_BUFFER_SIZE; ii++)
    {
      K30_CO2_data.msgData[ii] = _K30_I2C.read();
    }
    
  } else {
    if (_debug) { SERIAL_DEBUG.print("CO2: No data\n"); }
    _CO2_latest = 0;
    return;
  }

  // Invert the endianess on the CO2 reading
  K30_CO2_data.msg.CO2_value = invertEndian_16(K30_CO2_data.msg.CO2_value);

  // Check the checksum is valid
  uint8_t expected_checksum = checksum(K30_CO2_data.msgData, K30_CO2_RX_STREAM_BUFFER_SIZE - 1);
  if (K30_CO2_data.msg.checksum == expected_checksum)
  {
    _CO2_latest = K30_CO2_data.msg.CO2_value;
    return;
    
  } else {
    if (_debug)
    { 
      SERIAL_DEBUG.print("CO2: Checksum failed\n"); 
      SERIAL_DEBUG.print("\tExpected: 0x"); SERIAL_DEBUG.print(K30_CO2_data.msg.checksum); SERIAL_DEBUG.print("\n");
      SERIAL_DEBUG.print("\t     Got: 0x"); SERIAL_DEBUG.print(expected_checksum);         SERIAL_DEBUG.print("\n");
      
    }
    
    _CO2_latest = 0;
    return;
  }

  // We should never get this far
  _CO2_latest = 0;
  return;

}

uint16_t K30_CO2::get_CO2()
{
  // Returns the latest CO2 reading
  return (_CO2_latest);
}

uint8_t K30_CO2::checksum(uint8_t buf[], uint8_t len)
{
  uint8_t checksum = 0;

  // Calculate checksum by adding up the reading values
  for (uint8_t ii = 0; ii < len; ii++)
  {
    checksum = checksum + buf[ii];
  }

  // All done
  return (checksum);
}

uint16_t K30_CO2::invertEndian_16(uint16_t val)
{
  union val_union
  {
      uint16_t data16;
      uint8_t  data8[2];
  } inval;

  // Copy the data over
  inval.data16 = val;

  // Create the new data
  val_union retval;
  for (uint8_t ii=0; ii<sizeof(inval); ii++)
  {
    retval.data8[ii] = inval.data8[2-ii-1];
  }

  // Return the reverse value
  return (retval.data16);
  
}
