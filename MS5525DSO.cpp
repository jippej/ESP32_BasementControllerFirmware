#include "MS5525DSO.h"

#define MS5525DSO_CMD_RESET     ((uint8_t)0x1e)
#define MS5525DSO_CMD_BASE_PROM ((uint8_t)0xa0)
#define MS5525DSO_CMD_CONVERT   ((uint8_t)0x40)
#define MS5525DSO_CMD_READ_ADC  ((uint8_t)0x00)

#define P_SENS  (_PROM_coeff[0])
#define P_OFF   (_PROM_coeff[1])
#define TC_SENS (_PROM_coeff[2])
#define TC_OFF  (_PROM_coeff[3])
#define T_REF   (_PROM_coeff[4])
#define T_SENS  (_PROM_coeff[5])

/*##################################### PUBLIC FUNCTIONS #####################################*/

MS5525DSO::MS5525DSO(uint8_t csPin) : csPin(csPin) {}

void MS5525DSO::begin()
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, LOW);
  
  // Reset
  SPI.transfer(MS5525DSO_CMD_RESET);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  delay(10);
 
  // Read PROM calibration parameters 1 through 6
  for (uint8_t i = 1; i <= 6; i++) {
    _read_prom(i, &_PROM_coeff[i - 1]);
  }
}

void MS5525DSO::readPressureAndTemperature(double * MS5525DSO_pressure, double * MS5525DSO_temperature)
{
  uint32_t raw[2];  // index 0 is raw pressure, index 1 is raw temperature

  for (uint8_t i = 0; i < 2; i++) {
    _convert_D(i);

    // Delay the maximum expected time depending on OSR (4096 in this case)
    delay(10);

    _read_adc(&raw[i]);
  }

  // Difference between actual and reference temperature
  int64_t dT = raw[1] - ((int64_t)T_REF << 7);

  // Offset at actual temperature
  int64_t off = ((int64_t)P_OFF << 17) + ((TC_OFF * dT) >> 5);

  // Sensitivity at actual temperature
  int64_t sens = ((int64_t)P_SENS << 16) + ((TC_SENS * dT) >> 6);

  // Temperature compensated pressure
  int64_t tc_press = (((sens * raw[0]) >> 21) - off) >> 15;
  *MS5525DSO_pressure = tc_press * 0.0001;

  // Calculate cooked temperature if requested
  if (MS5525DSO_temperature != NULL) {
    *MS5525DSO_temperature = (2000 + ((dT * T_SENS) >> 21)) * 0.01;
  }

}

/*##################################### PRIVAT FUNCTIONS #####################################*/

void MS5525DSO::_read_prom(uint8_t i, uint16_t * c)
{
  // Address of PROM parameter to read
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(MS5525DSO_CMD_BASE_PROM | (i << 1));


  // Read 2 bytes of the coefficient, MSB first
  *c = 0;
  for (auto n = 0; n < 2; n++) {
    *c = (*c << 8) | SPI.transfer(0);
  }
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  
}

void MS5525DSO::_convert_D(uint8_t d)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(MS5525DSO_CMD_CONVERT | ((d & 0x01) << 4) | 8); // 8 is OSR value 4096
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

void MS5525DSO::_read_adc(uint32_t * c)
{
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(MS5525DSO_CMD_READ_ADC);
  
  // Read 3 bytes of the ADC result, MSB first
  *c = 0;
  for (auto n = 0; n < 3; n++) {
    *c = (*c << 8) | SPI.transfer(0);
  }

  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}
