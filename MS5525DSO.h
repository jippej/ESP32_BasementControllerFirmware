/**
 * Library for the MS5525DSO series pressure sensor from MEAS.
 * 
 * by J.Dekker for SPI only communication
 *
 */
 
#ifndef __MS5525DSO_H__
#define __MS5525DSO_H__

#include "Arduino.h"
#include <SPI.h>

class MS5525DSO {
public:
  // Constructor
  MS5525DSO(uint8_t csPin);

  // call in setup
  void begin();

  void readPressureAndTemperature(double * MS5525DSO_pressure, double * MS5525DSO_temperature);

private:
  // private variables
  uint8_t csPin;
  uint16_t _PROM_coeff[6];
  
  void _read_prom(uint8_t i, uint16_t * c);
  void _convert_D(uint8_t d);
  void _read_adc(uint32_t * c);

};

#endif
