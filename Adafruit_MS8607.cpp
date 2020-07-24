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
  bool success = false;
  uint8_t tmp_buffer[2];

  for (int i = 0; i < 7; i++) {
    offset = 2 * i;
    tmp_buffer[0] = PROM_ADDRESS_READ_ADDRESS_0 + offset;
    success = success && i2c_dev->write_then_read(tmp_buffer, 1, tmp_buffer, 2);
    buffer[i] = tmp_buffer[0] << 8;
    buffer[i] |= tmp_buffer[1];
  }
  if (success != 0) {
    return success;
  }

  if (!_psensor_crc_check(buffer, (buffer[0] & 0xF000) >> 12))
    return false;

  return true;
}

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
