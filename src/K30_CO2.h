// Library for reading KC30 CO2 sensor
//
// Matt Anderson, 2020

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "data_structures.h"

#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG Serial
#endif

class K30_CO2 {
    public :
        K30_CO2(TwoWire& K30_I2C, int CS_CALIB);
        
        void init();
        void calibrate();
        unsigned long t_calibrate();
        void update();
        uint16_t get_CO2();

        uint8_t _debug = 0;
        
    private :
        TwoWire& _K30_I2C;
        int _CS_CALIB;

        uint8_t _K30_ADDR = 0x68;
        
        uint16_t _CO2_latest = 0;
        bool _enabled = 0;
        
        const unsigned long _t_CO2_calibrate = 10UL*1000; // Time required to pull the calibrate pin high on the CO2 sensor to run the calibration in [ ms ]
        
        // Checksumming
        uint8_t checksum(uint8_t buf[], uint8_t len);

        // Helpers
        uint16_t invertEndian_16(uint16_t val);
        
};
