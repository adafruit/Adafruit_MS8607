/*!
 *  @file Adafruit_MS8607.cpp
 *
 *  This is a library for the MS8607 
 *
 *  Designed specifically to work with the Adafruit MS8607 Pressure, Humidity, and Temperature Sensor.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/XXXX
 *
 *  These sensors use XXX to communicate, X pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Bryan Siepert (Adafruit Industries)
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_MS8607.h"
#include "Arduino.h"
#include <Wire.h>

/*!
 * @brief  MS8607 constructor using i2c
 * @param  *theWire
 *         optional wire
 */
Adafruit_MS8607::Adafruit_MS8607(TwoWire *theWire)
    : _cs(-1), _mosi(-1), _miso(-1), _sck(-1) {
  _wire = theWire;
  pt_sensor = new Adafruit_MS8607_PT(this);
  rh_sensor = new Adafruit_MS8607_RH(this);
}

Adafruit_MS8607::~Adafruit_MS8607(void) {
  delete pt_sensor;
  delete rh_sensor;
}

/*!
 * @brief  MS8607 constructor using hardware SPI
 * @param  cspin
 *         cs pin number
 * @param  theSPI
 *         optional SPI object
 */
Adafruit_MS8607::Adafruit_MS8607(int8_t cspin, SPIClass *theSPI)
    : _cs(cspin), _mosi(-1), _miso(-1), _sck(-1) {
  _spi = theSPI;
}

/*!
 * @brief  MS8607 constructor using bitbang SPI
 * @param  cspin
 *         The pin to use for CS/SSEL.
 * @param  mosipin
 *         The pin to use for MOSI.
 * @param  misopin
 *         The pin to use for MISO.
 * @param  sckpin
 *         The pin to use for SCK.
 */
Adafruit_MS8607::Adafruit_MS8607(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin)
    : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin) {}

/*!
 *  Initialises the sensor.
 *  @param addr
 *         The I2C address to use (default = 0x77)
 *  @param chipid
 *         The expected chip ID (used to validate connection).
 *  @return True if the init was successful, otherwise false.
 */
bool Adafruit_MS8607::begin(uint8_t addr, uint8_t chipid) {
  _i2caddr = addr;

  if (_cs == -1) {
    // i2c
    _wire->begin();
  } else {
    digitalWrite(_cs, HIGH);
    pinMode(_cs, OUTPUT);

    if (_sck == -1) {
      // hardware SPI
      _spi->begin();
    } else {
      // software SPI
      pinMode(_sck, OUTPUT);
      pinMode(_mosi, OUTPUT);
      pinMode(_miso, INPUT);
    }
  }

  if (read8(MS8607_REGISTER_CHIPID) != chipid)
    return false;

  readCoefficients();
  // write8(MS8607_REGISTER_CONTROL, 0x3F); /* needed? */
  setSampling();
  delay(100);
  return true;
}


/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void Adafruit_MS8607::write8(byte reg, byte value) {
  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->write((uint8_t)value);
    _wire->endTransmission();
  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg & ~0x80); // write, bit 7 low
    spixfer(value);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }
}

/*!
 *  @brief  Reads an 8 bit value over I2C/SPI
 *  @param  reg
 *          selected register
 *  @return value from selected register
 */
uint8_t Adafruit_MS8607::read8(byte reg) {
  uint8_t value;

  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2caddr, (byte)1);
    value = _wire->read();

  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }
  return value;
}

/*!
 *  @brief  Reads a 16 bit value over I2C/SPI
 */
uint16_t Adafruit_MS8607::read16(byte reg) {
  uint16_t value;

  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2caddr, (byte)2);
    value = (_wire->read() << 8) | _wire->read();

  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high
    value = (spixfer(0) << 8) | spixfer(0);
    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }

  return value;
}

uint16_t Adafruit_MS8607::read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

/*!
 *   @brief  Reads a signed 16 bit value over I2C/SPI
 */
int16_t Adafruit_MS8607::readS16(byte reg) { return (int16_t)read16(reg); }

int16_t Adafruit_MS8607::readS16_LE(byte reg) {
  return (int16_t)read16_LE(reg);
}

/*!
 *  @brief  Reads a 24 bit value over I2C/SPI
 */
uint32_t Adafruit_MS8607::read24(byte reg) {
  uint32_t value;

  if (_cs == -1) {
    _wire->beginTransmission((uint8_t)_i2caddr);
    _wire->write((uint8_t)reg);
    _wire->endTransmission();
    _wire->requestFrom((uint8_t)_i2caddr, (byte)3);

    value = _wire->read();
    value <<= 8;
    value |= _wire->read();
    value <<= 8;
    value |= _wire->read();

  } else {
    if (_sck == -1)
      _spi->beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);
    spixfer(reg | 0x80); // read, bit 7 high

    value = spixfer(0);
    value <<= 8;
    value |= spixfer(0);
    value <<= 8;
    value |= spixfer(0);

    digitalWrite(_cs, HIGH);
    if (_sck == -1)
      _spi->endTransaction(); // release the SPI bus
  }

  return value;
}

/*!
 *  @brief  Reads the factory-set coefficients
 */
void Adafruit_MS8607::readCoefficients() {
  _ms8607_calib.SENS_T1 = read16_LE(MS8607_REGISTER_SENS_T1);
  _ms8607_calib.OFF_T1 = readS16_LE(MS8607_REGISTER_OFF_T1);
  _ms8607_calib.TCS = readS16_LE(MS8607_REGISTER_TCS);
  _ms8607_calib.TCO = read16_LE(MS8607_REGISTER_TCO);
  _ms8607_calib.T_REF = readS16_LE(MS8607_REGISTER_T_REF);
  _ms8607_calib.TEMPSENS = readS16_LE(MS8607_REGISTER_TEMPSENS);
}

/*!
 *  @brief  Resets the chip via soft reset
 */
void Adafruit_MS8607::reset(void) {
  write8(MS8607_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE);
}

/*!
    @brief  Gets the most recent sensor event from the hardware status register.
    @return Sensor status as a byte.
 */
uint8_t Adafruit_MS8607::getStatus(void) {
  return read8(MS8607_REGISTER_STATUS);
}
