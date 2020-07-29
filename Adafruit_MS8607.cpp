/*!
 *  @file Adafruit_MS8607.cpp
 *
 *  This is a library for the MS8607
 *
 *  Designed specifically to work with the Adafruit MS8607 Pressure, Humidity,
 * and Temperature Sensor.
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
#include <Adafruit_MS8607.h>
// enum MS8607_status MS8607::hsensor_reset(void)
// {
//   // enum MS8607_status status;

//   _i2cPort->beginTransmission((uint8_t)MS8607_HSENSOR_ADDR);
//   _i2cPort->write((uint8_t)HSENSOR_RESET_COMMAND);
//   _i2cPort->endTransmission();

//   // if (status != MS8607_status_ok)
//   //   return status;

//   // hsensor_conversion_time = HSENSOR_CONVERSION_TIME_12b;
//   // delay(HSENSOR_RESET_TIME);

//   // return MS8607_status_ok;
// }

/*!
 *    @brief  Instantiates a new MS8607 class
 */
Adafruit_MS8607::Adafruit_MS8607(void) {}
Adafruit_MS8607::~Adafruit_MS8607(void) {
  // if (temp_sensor) {
  //   delete temp_sensor;
  // }
  // if (humidity_sensor) {
  //   delete humidity_sensor;
  // }
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_MS8607::begin(uint8_t i2c_address, TwoWire *wire,
                            int32_t sensor_id) {

  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }
  // return reset();
  return _init(sensor_id);
}

/**
 * @brief Reset the sensors to their initial state
 *
 * @return true: success false: failure
 */
bool Adafruit_MS8607::reset(void) {
  uint8_t cmd = P_T_RESET;
  return i2c_dev->write(&cmd, 1);
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_MS8607::_init(int32_t sensor_id) {
  uint8_t offset = 0;
  uint16_t buffer[7];
  uint8_t tmp_buffer[2];

  for (int i = 0; i < 7; i++) {
    offset = 2 * i;
    tmp_buffer[0] = PROM_ADDRESS_READ_ADDRESS_0 + offset;
    i2c_dev->write_then_read(tmp_buffer, 1, tmp_buffer, 2);
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
  psensor_resolution_osr = MS8607_PRESSURE_RESOLUTION_OSR_8192;

  return true;
}

/**
 * @brief Read the current pressure and temperature
 *
 * @return true: success false: failure
 */
bool Adafruit_MS8607::_read(void) {

  int32_t dT, TEMP;
  int64_t OFF, SENS, P, T2, OFF2, SENS2;
  uint8_t cmd;

  uint8_t status;
  uint8_t buffer[3];

  uint32_t raw_temp, raw_pressure;

  // First read temperature
  cmd = psensor_resolution_osr * 2;
  cmd |= PSENSOR_START_TEMPERATURE_ADC_CONVERSION;
  status |= i2c_dev->write(&cmd, 1);

  // 20ms wait for conversion
  // TODO: change delay depending on resolution
  // delay(psensor_conversion_time[psensor_resolution_osr]);
  delay(18);

  buffer[0] = PSENSOR_READ_ADC;
  status |= i2c_dev->write_then_read(buffer, 1, buffer, 3);

  raw_temp =
      ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

  // Now read pressure
  cmd = psensor_resolution_osr * 2; // ( << 1) OSR = USER[6
  cmd |= PSENSOR_START_PRESSURE_ADC_CONVERSION;
  status |= i2c_dev->write(&cmd, 1);
  delay(18);

  buffer[0] = PSENSOR_READ_ADC;
  status |= i2c_dev->write_then_read(buffer, 1, buffer, 3);

  raw_pressure =
      ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

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

  temperature = ((float)TEMP - T2) / 100;
  pressure = (float)P / 100;

  return status;

  //////////////////////////////////
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
