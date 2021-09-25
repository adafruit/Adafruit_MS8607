/*!
 *  @file Adafruit_MS8607.cpp
 *

 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Author:
 *  Bryan Siepert for Adafruit Industries
 *
 * Parts of this library were adapted by Bryan Siepert for Adafruit Industries,
 July 31,2020 from
 * the Sparkfun_MS8607_Arduino_Library code. The License and Copyright info from
 that library
 * follows:

 * This is a library written for the MS8607. Originally written by
 TEConnectivity
 * with an MIT license. Library updated and brought to fit Arduino Library
 standards
 * by PaulZC, October 30th, 2019. Based extensively on the
 Sparkfun_MS5637_Arduino_Library
 * by Nathan Seidle @ SparkFun Electronics, April 13th, 2018.

 * The MS8607 is a combined pressure, humidity and temperature sensor. The
 pressure
 * sensor will operate down to 10mbar which is equivalent to an altitude of
 approx.
 * 31,000m making it suitable for high altitude ballooning as well as many other
 * weather station applications.

 * This library handles the initialization of the MS8607 and is able to
 * query the sensor for different readings.

 * Development environment specifics:
 * Arduino IDE 1.8.10
 *

 *  MIT License

 * Copyright (c) 2016 TE Connectivity

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.

 */

#include <Adafruit_MS8607.h>

/*!
 *    @brief  Instantiates a new MS8607 class
 */
Adafruit_MS8607::Adafruit_MS8607(void) {}
Adafruit_MS8607::~Adafruit_MS8607(void) {
  if (temp_sensor) {
    delete temp_sensor;
  }
  if (pressure_sensor) {
    delete pressure_sensor;
  }
  if (humidity_sensor) {
    delete humidity_sensor;
  }
  if (pt_i2c_dev) {
    delete pt_i2c_dev;
  }

  if (hum_i2c_dev) {
    delete hum_i2c_dev;
  }
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_MS8607::begin(TwoWire *wire, int32_t sensor_id) {

  if (pt_i2c_dev) {
    delete pt_i2c_dev;
  }

  if (hum_i2c_dev) {
    delete hum_i2c_dev;
  }

  pt_i2c_dev = new Adafruit_I2CDevice(MS8607_PT_ADDRESS, wire);
  hum_i2c_dev = new Adafruit_I2CDevice(MS8607_HUM_ADDRESS, wire);

  if (!pt_i2c_dev->begin()) {
    return false;
  }
  if (!hum_i2c_dev->begin()) {
    return false;
  }
  reset();
  temp_sensor = new Adafruit_MS8607_Temp(this);
  pressure_sensor = new Adafruit_MS8607_Pressure(this);
  humidity_sensor = new Adafruit_MS8607_Humidity(this);

  return init(sensor_id);
}

/**
 * @brief Reset the sensors to their initial state
 *
 * @return true: success false: failure
 */
bool Adafruit_MS8607::reset(void) {
  uint8_t cmd = P_T_RESET;
  bool success = true;
  success = pt_i2c_dev->write(&cmd, 1);
  if (!success) {
    return false;
  }

  cmd = HSENSOR_RESET_COMMAND;
  success = hum_i2c_dev->write(&cmd, 1);
  if (!success) {
    return false;
  }
  delay(15);
  return success;
}

bool Adafruit_MS8607::_fetch_temp_calibration_values(void) {
  uint8_t offset = 0;
  uint16_t buffer[8];
  uint8_t tmp_buffer[2];

  for (int i = 0; i < 7; i++) {
    offset = 2 * i;
    tmp_buffer[0] = PROM_ADDRESS_READ_ADDRESS_0 + offset;
    pt_i2c_dev->write_then_read(tmp_buffer, 1, tmp_buffer, 2);
    buffer[i] = tmp_buffer[0] << 8;
    buffer[i] |= tmp_buffer[1];
  }

  if (!_psensor_crc_check(buffer, (buffer[0] & 0xF000) >> 12)) {
    return false;
  }
  press_sens = buffer[1];
  press_offset = buffer[2];
  press_sens_temp_coeff = buffer[3];
  press_offset_temp_coeff = buffer[4];
  ref_temp = buffer[5];
  temp_temp_coeff = buffer[6];
  return true;
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_MS8607::init(int32_t sensor_id) {

  if (!_fetch_temp_calibration_values()) {
    return false;
  }
  if (!enableHumidityClockStretching(false)) {
    return false;
  }
  if (!setHumidityResolution(MS8607_HUMIDITY_RESOLUTION_OSR_12b)) {
    return false;
  }
  if (!setPressureResolution(MS8607_PRESSURE_RESOLUTION_OSR_4096)) {
    return false;
  }

  return true;
}

/**
 * @brief Get the currently set resolution for humidity readings
 *
 * @return ms8607_humidity_resolution_t the current resolution
 */
ms8607_humidity_resolution_t Adafruit_MS8607::getHumidityResolution(void) {
  uint8_t reg_value = _read_humidity_user_register();

  return ((ms8607_humidity_resolution_t)(reg_value &
                                         HSENSOR_USER_REG_RESOLUTION_MASK));
}
/**
 * @brief Set the resolution for humidity readings
 *
 * @param resolution The new resolution to set
 * @return true: success false: failure
 */
bool Adafruit_MS8607::setHumidityResolution(
    ms8607_humidity_resolution_t resolution) {
  uint8_t reg_value = _read_humidity_user_register();

  // unset current value
  reg_value &= ~HSENSOR_USER_REG_RESOLUTION_MASK;
  // set new value
  reg_value |= resolution & HSENSOR_USER_REG_RESOLUTION_MASK;

  return _write_humidity_user_register(reg_value);
}
/**
 * @brief Get the currently set resolution for pressure readings
 *
 * @return ms8607_pressure_resolution_t the current resolution
 */
ms8607_pressure_resolution_t Adafruit_MS8607::getPressureResolution(void) {
  return psensor_resolution_osr;
}

/**
 * @brief Set the resolution for pressure readings
 *
 * @param resolution The new resolution to set
 * @return true: success false: failure
 */
bool Adafruit_MS8607::setPressureResolution(
    ms8607_pressure_resolution_t resolution) {
  psensor_resolution_osr = resolution;
  return true;
}

/**
 * @brief Allow the MS8607 to hold the clock line low until it completes the
 * requested measurements
 *
 * @param enable_stretching true: enable
 * @return true: success false: failure
 */
bool Adafruit_MS8607::enableHumidityClockStretching(bool enable_stretching) {
  if (enable_stretching) {
    _hum_sensor_i2c_read_mode = MS8607_I2C_HOLD;
  } else {
    _hum_sensor_i2c_read_mode = MS8607_I2C_NO_HOLD;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the humidity sensor and temperature values as sensor events
    @param  pressure Sensor event object that will be populated with pressure
   data
    @param  temp Sensor event object that will be populated with temp data
    @param  humidity Sensor event object that will be populated with humidity
   data
    @returns true if the event data was read successfully
*/
/**************************************************************************/
bool Adafruit_MS8607::getEvent(sensors_event_t *pressure, sensors_event_t *temp,
                               sensors_event_t *humidity) {
  uint32_t t = millis();

  _read();
  // use helpers to fill in the events
  if (temp)
    fillTempEvent(temp, t);
  if (pressure)
    fillPressureEvent(pressure, t);
  if (humidity) {
    _read_humidity();
    fillHumidityEvent(humidity, t);
  }
  return true;
}

/**
 * @brief  Gets the sensor_t object describing the MS8607's tenperature sensor
 *
 * @param sensor The sensor_t object to be populated
 */
void Adafruit_MS8607_Temp::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MS8607_T", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40;
  sensor->max_value = 85;
  sensor->resolution = 0.01; // depends on calibration data?
}

/**
 * @brief  Gets the sensor_t object describing the MS8607's pressure sensor
 *
 * @param sensor The sensor_t object to be populated
 */
void Adafruit_MS8607_Pressure::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MS8607_P", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_PRESSURE;
  sensor->min_delay = 0;
  sensor->min_value = 10;
  sensor->max_value = 2000;
  sensor->resolution = 0.016;
}

/**
 * @brief  Gets the sensor_t object describing the MS8607's humidity sensor
 *
 * @param sensor The sensor_t object to be populated
 */
void Adafruit_MS8607_Humidity::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "MS8607_H", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sensor->min_delay = 0;
  sensor->min_value = 0;
  sensor->max_value = 100;
  sensor->resolution = 0.04;
}
/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns true
*/
bool Adafruit_MS8607_Temp::getEvent(sensors_event_t *event) {
  _theMS8607->getEvent(NULL, event, NULL);

  return true;
}
/*!
    @brief  Gets the pressure as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns true
*/
bool Adafruit_MS8607_Pressure::getEvent(sensors_event_t *event) {
  _theMS8607->getEvent(event, NULL, NULL);

  return true;
}
/*!
    @brief  Gets the relative humidity as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns true
*/
bool Adafruit_MS8607_Humidity::getEvent(sensors_event_t *event) {
  _theMS8607->getEvent(NULL, NULL, event);

  return true;
}
/**
 * @brief Read the current pressure and temperature
 *
 * @return true: success false: failure
 */
bool Adafruit_MS8607::_read(void) {

  uint8_t cmd;
  bool status = true;
  uint8_t buffer[3];

  uint32_t raw_temp, raw_pressure;

  // First read temperature
  cmd = psensor_resolution_osr * 2;
  cmd |= PSENSOR_START_TEMPERATURE_ADC_CONVERSION;
  status = pt_i2c_dev->write(&cmd, 1);
  if (!status) {
    return status;
  }

  delay(18);

  buffer[0] = PSENSOR_READ_ADC;
  status = pt_i2c_dev->write_then_read(buffer, 1, buffer, 3);
  if (!status) {
    return status;
  }
  raw_temp =
      ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

  // Now read pressure
  cmd = psensor_resolution_osr * 2;
  cmd |= PSENSOR_START_PRESSURE_ADC_CONVERSION;
  status = pt_i2c_dev->write(&cmd, 1);
  delay(18);

  buffer[0] = PSENSOR_READ_ADC;
  status = pt_i2c_dev->write_then_read(buffer, 1, buffer, 3);

  raw_pressure =
      ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

  return _applyPTCorrections(raw_temp, raw_pressure);
}

bool Adafruit_MS8607::_applyPTCorrections(int32_t raw_temp,
                                          int32_t raw_pressure) {
  int32_t dT, TEMP;
  int64_t OFF, SENS, P, T2, OFF2, SENS2;
  dT = (int32_t)raw_temp - ((int32_t)ref_temp << 8);

  // Actual temperature = 2000 + dT * TEMPSENS
  TEMP = 2000 + ((int64_t)dT * (int64_t)temp_temp_coeff >> 23);

  // Second order temperature compensation
  if (TEMP < 2000) {
    T2 = (3 * ((int64_t)dT * (int64_t)dT)) >> 33;
    OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16;
    SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16;

    if (TEMP < -1500) {
      OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
      SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
    }
  } else {
    T2 = (5 * ((int64_t)dT * (int64_t)dT)) >> 38;
    OFF2 = 0;
    SENS2 = 0;
  }

  // OFF = OFF_T1 + TCO * dT
  OFF = ((int64_t)(press_offset) << 17) +
        (((int64_t)press_offset_temp_coeff * dT) >> 6);
  OFF -= OFF2;

  // Sensitivity at actual temperature = SENS_T1 + TCS * dT
  SENS = ((int64_t)press_sens << 16) +
         (((int64_t)press_sens_temp_coeff * dT) >> 7);
  SENS -= SENS2;

  // Temperature compensated pressure = D1 * SENS - OFF
  P = (((raw_pressure * SENS) >> 21) - OFF) >> 15;

  _temperature = ((float)TEMP - T2) / 100;
  _pressure = (float)P / 100;

  return true;
}
/*
humidity user register value: 0b10
humidity resolution raw value: 0x0
Temperature: 29.85 degrees C
Pressure: 1008.94 hPa
Relative Humidity: 25.94 %rH
*/
bool Adafruit_MS8607::_read_humidity(void) {
  uint8_t buffer[3];
  getHumidityResolution();
  buffer[0] = MS8607_I2C_NO_HOLD;
  // self._buffer[0] = _MS8607_HUM_CMD_READ_NO_HOLD
  hum_i2c_dev->write(buffer, 1);
  delay(20);
  hum_i2c_dev->read(buffer, 3);

  uint16_t raw_hum = buffer[0] << 8 | buffer[1];
  uint8_t crc = buffer[2];
  if (!_hsensor_crc_check(raw_hum, crc)) {
    return false;
  }
  _humidity = raw_hum * MS8607_RH_LSB;
  _humidity -= 6;
  return true;
}

/********************* Sensor Methods ****************************************/
/**
 * @brief Gets the Adafruit_Sensor object for the MS0607's temperature sensor
 * @return Adafruit_Sensor* a pointer to the temperature sensor object
 */
Adafruit_Sensor *Adafruit_MS8607::getTemperatureSensor(void) {
  return temp_sensor;
}

/**
 * @brief Gets the Adafruit_Sensor object for the MS0607's pressure sensor
 * @return Adafruit_Sensor* a pointer to the pressure sensor object
 */
Adafruit_Sensor *Adafruit_MS8607::getPressureSensor(void) {
  return pressure_sensor;
}

/**
 * @brief Gets the Adafruit_Sensor object for the MS0607's humidity sensor
 * @return Adafruit_Sensor* a pointer to the humidity sensor object
 */
Adafruit_Sensor *Adafruit_MS8607::getHumiditySensor(void) {
  return humidity_sensor;
}
void Adafruit_MS8607::fillHumidityEvent(sensors_event_t *humidity,
                                        uint32_t timestamp) {
  memset(humidity, 0, sizeof(sensors_event_t));
  humidity->version = sizeof(sensors_event_t);
  humidity->sensor_id = _sensorid_humidity;
  humidity->type = SENSOR_TYPE_PRESSURE;
  humidity->timestamp = timestamp;
  humidity->relative_humidity = _humidity;
}
void Adafruit_MS8607::fillTempEvent(sensors_event_t *temp, uint32_t timestamp) {
  memset(temp, 0, sizeof(sensors_event_t));
  temp->version = sizeof(sensors_event_t);
  temp->sensor_id = _sensorid_temp;
  temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->timestamp = timestamp;
  temp->temperature = _temperature;
}

void Adafruit_MS8607::fillPressureEvent(sensors_event_t *pressure,
                                        uint32_t timestamp) {
  memset(pressure, 0, sizeof(sensors_event_t));
  pressure->version = sizeof(sensors_event_t);
  pressure->sensor_id = _sensorid_pressure;
  pressure->type = SENSOR_TYPE_PRESSURE;
  pressure->timestamp = timestamp;
  pressure->pressure = _pressure;
}
/***************************  Private Methods *********************************/
bool Adafruit_MS8607::_psensor_crc_check(uint16_t *n_prom, uint8_t crc) {
  uint8_t cnt, n_bit;
  uint16_t n_rem, crc_read;

  n_rem = 0x00;
  crc_read = n_prom[0];
  n_prom[7] = 0;
  n_prom[0] = (0x0FFF & (n_prom[0])); // Clear the CRC byte

  for (cnt = 0; cnt < (7 + 1) * 2; cnt++) {

    // Get next byte
    if (cnt % 2 == 1)
      n_rem ^= n_prom[cnt >> 1] & 0x00FF;
    else
      n_rem ^= n_prom[cnt >> 1] >> 8;

    for (n_bit = 8; n_bit > 0; n_bit--) {

      if (n_rem & 0x8000)
        n_rem = (n_rem << 1) ^ 0x3000;
      else
        n_rem <<= 1;
    }
  }
  n_rem >>= 12;
  n_prom[0] = crc_read;
  return (n_rem == crc);
}

bool Adafruit_MS8607::_hsensor_crc_check(uint16_t value, uint8_t crc) {

  uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
  uint32_t msb = 0x800000;
  uint32_t mask = 0xFF8000;
  uint32_t result = (uint32_t)value << 8; // Pad with zeros as specified in spec

  while (msb != 0x80) {

    // Check if msb of current value is 1 and apply XOR mask
    if (result & msb)
      result = ((result ^ polynom) & mask) | (result & ~mask);

    // Shift by one
    msb >>= 1;
    mask >>= 1;
    polynom >>= 1;
  }
  return (result == crc);
}

uint8_t Adafruit_MS8607::_read_humidity_user_register(void) {
  uint8_t buffer = HSENSOR_READ_USER_REG_COMMAND;
  hum_i2c_dev->write_then_read(&buffer, 1, &buffer, 1, true);

  return buffer;
}

bool Adafruit_MS8607::_write_humidity_user_register(uint8_t new_reg_value) {
  uint8_t buffer[2];
  buffer[0] = HSENSOR_WRITE_USER_REG_COMMAND;
  buffer[1] = new_reg_value;
  return hum_i2c_dev->write(buffer, 2);
}
