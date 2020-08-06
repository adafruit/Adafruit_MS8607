/*!
 *  @file Adafruit_MS8607.h
 *  @mainpage
 *
 *  This is a library for the MS8607 Pressure, Temperature, and Humidity Sensor
 from TE Connectivity
 *
 *  Designed specifically to work with the Adafruit MS8607 Pressure, Humidity,
 * and Temperature Sensor.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/XXXX
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
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
 * @section license License
 *  MIT License
 *
 * @section copyright Copyright

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

#ifndef __MS8607_H__
#define __MS8607_H__

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/*!
 *  I2C ADDRESS/BITS/SETTINGS. The MS8607 uses two different I2C addresses
 */
#define MS8607_PT_ADDRESS                                                      \
  0x76 ///< The pressure and temperature I2C address for the sensor
#define MS8607_HUM_ADDRESS                                                     \
  0x40 ///< The default pressure and temperature I2C address for the sensor

// HSENSOR device commands
#define HSENSOR_RESET_COMMAND 0xFE ///< reset command
#define HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND                                   \
  0xE5 ///< read humidity w hold command
#define HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND                                  \
  0xF5 ///< read humidity wo hold command
#define HSENSOR_READ_SERIAL_FIRST_8BYTES_COMMAND                               \
  0xFA0F ///< read serial first 8bytes command
#define HSENSOR_READ_SERIAL_LAST_6BYTES_COMMAND                                \
  0xFCC9                                    ///< read serial last 6bytes command
#define HSENSOR_WRITE_USER_REG_COMMAND 0xE6 ///< write user reg command
#define HSENSOR_READ_USER_REG_COMMAND 0xE7  ///< read user reg command

// Processing constants
#define HSENSOR_TEMPERATURE_COEFFICIENT                                        \
  (float)(-0.15)                            ///< temperature coefficient
#define HSENSOR_CONSTANT_A (float)(8.1332)  ///< constant a
#define HSENSOR_CONSTANT_B (float)(1762.39) ///< constant b
#define HSENSOR_CONSTANT_C (float)(235.66)  ///< constant c

// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL (175.72) ///< temperature coeff mul
#define TEMPERATURE_COEFF_ADD (-46.85) ///< temperature coeff add

// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL (125) ///< humidity coeff mul
#define HUMIDITY_COEFF_ADD (-6)  ///< humidity coeff add

// Conversion timings
#define HSENSOR_CONVERSION_TIME_12b 16 ///< conversion time 12b
#define HSENSOR_CONVERSION_TIME_10b 5  ///< conversion time 10b
#define HSENSOR_CONVERSION_TIME_8b 3   ///< conversion time 8b
#define HSENSOR_CONVERSION_TIME_11b 9  ///< conversion time 11b

// HSENSOR User Register masks and bit position
#define HSENSOR_USER_REG_RESOLUTION_MASK 0x81 ///< user reg resolution mask
#define HSENSOR_USER_REG_END_OF_BATTERY_MASK                                   \
  0x40 ///< user reg end of battery mask
#define HSENSOR_USER_REG_ENABLE_ONCHIP_HEATER_MASK                             \
  0x4 ///< user reg enable onchip heater mask
#define HSENSOR_USER_REG_DISABLE_OTP_RELOAD_MASK                               \
  0x2 ///< user reg disable otp reload mask

#define MS8607_RH_ADDRESS (0x40) /**< Humidity I2C address for the sensor. */

// PSENSOR commands
#define PROM_ADDRESS_READ_ADDRESS_0 0xA0 ///< 16-bit registers through 0xAE

/* Pressure & Temperature commands */
#define P_T_RESET 0x1E           //!< Pressure and temperature sensor reset
#define CONVERT_D1_OSR_256 0x40  //!< Sampling rate 256
#define CONVERT_D1_OSR_512 0x42  //!< Sampling rate 512
#define CONVERT_D1_OSR_1024 0x44 //!< Sampling rate 1024
#define CONVERT_D1_OSR_2048 0x46 //!< Sampling rate 2048
#define CONVERT_D1_OSR_4096 0x48 //!< Sampling rate 4096
#define CONVERT_D1_OSR_8192 0x4A //!< Sampling rate 8192
#define CONVERT_D2_OSR_256 0x50  //!< Sampling rate 256
#define CONVERT_D2_OSR_512 0x52  //!< Sampling rate 512
#define CONVERT_D2_OSR_1024 0x54 //!< Sampling rate 1024
#define CONVERT_D2_OSR_2048 0x56 //!< Sampling rate 2048
#define CONVERT_D2_OSR_4096 0x58 //!< Sampling rate 4096
#define CONVERT_D2_OSR_8192 0x5A //!< Sampling rate 8192
#define ADC_READ 0x00            //!< Command to read from ADC
/* PROM READ P&T is from 0xA0 to 0xAE. Not sure whether or not to add */

/* Commands for relative humidity */
#define HUM_RESET 0xFE          //!< Humidity sensor reset
#define HUM_WRITE_REGISTER 0xE6 //!< Humidity sensor write register
#define HUM_READ_REGISTER 0xE7  //!< Humidity sensor read register
#define HUM_MEASURE_RH_HOLD                                                    \
  0xE5 //!< Humidity sensor measure relative humidity hold master
#define HUM_MEASURE_RH_NO_HOLD                                                 \
  0xF5 //!< Humidity sensor measure relative humidity no hold master
/* PROM READ RH is from 0xA0 to 0xAE. Not sure whether or not to add */

#define PSENSOR_RESET_COMMAND 0x1E ///< Command to reset pressure sensor
#define PSENSOR_START_PRESSURE_ADC_CONVERSION                                  \
  0x40 ///< Command to start pressure ADC measurement
#define PSENSOR_START_TEMPERATURE_ADC_CONVERSION                               \
  0x50                        ///< Command to start temperature ADC measurement
#define PSENSOR_READ_ADC 0x00 ///< Temp and pressure ADC read command

/**
 * @brief Pressure sensor resolution options
 *
 */
typedef enum {
  MS8607_PRESSURE_RESOLUTION_OSR_256,  ///< 0
  MS8607_PRESSURE_RESOLUTION_OSR_512,  ///< 1
  MS8607_PRESSURE_RESOLUTION_OSR_1024, ///< 2
  MS8607_PRESSURE_RESOLUTION_OSR_2048, ///< 3
  MS8607_PRESSURE_RESOLUTION_OSR_4096, ///< 4
  MS8607_PRESSURE_RESOLUTION_OSR_8192, ///< 5
} ms8607_pressure_resolution_t;

/**
 * @brief Options for setHumidityResolution
 *
 */
typedef enum {
  MS8607_HUMIDITY_RESOLUTION_OSR_12b = 0x00,
  MS8607_HUMIDITY_RESOLUTION_OSR_11b = 0x81,
  MS8607_HUMIDITY_RESOLUTION_OSR_10b = 0x80,
  MS8607_HUMIDITY_RESOLUTION_OSR_8b = 0x01,
} ms8607_humidity_resolution_t;

/**
 * @brief Options for I2C clock stretch for humidity readings
 *
 */
typedef enum {
  MS8607_I2C_HOLD = 0xE5,
  MS8607_I2C_NO_HOLD = 0xF5,
} ms8607_hum_clock_stretch_t;

class Adafruit_MS8607;

#define HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND                                   \
  0xE5 ///< read humidity w hold command
#define HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND                                  \
  0xF5 ///< read humidity wo hold command
/**
 * @brief Adafruit Unified Sensor interface for the temperature sensor component
 * of the MS8607
 *
 */
class Adafruit_MS8607_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temperature
     sensor
      @param parent A pointer to the MS8607 class */
  Adafruit_MS8607_Temp(Adafruit_MS8607 *parent) { _theMS8607 = parent; }

  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x8600;
  Adafruit_MS8607 *_theMS8607 = NULL;
};
/**
 * @brief Adafruit Unified Sensor interface for the pressure sensor component
 * of the MS8607
 *
 */
class Adafruit_MS8607_Pressure : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the Pressure sensor
      @param parent A pointer to the MS8607 class */
  Adafruit_MS8607_Pressure(Adafruit_MS8607 *parent) { _theMS8607 = parent; }

  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x8601;
  Adafruit_MS8607 *_theMS8607 = NULL;
};

/**
 * @brief Adafruit Unified Sensor interface for the pressure sensor component
 * of the MS8607
 *
 */
class Adafruit_MS8607_Humidity : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the Humidity sensor
      @param parent A pointer to the MS8607 class */
  Adafruit_MS8607_Humidity(Adafruit_MS8607 *parent) { _theMS8607 = parent; }

  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x8602;
  Adafruit_MS8607 *_theMS8607 = NULL;
};

/**
 * Driver for the Adafruit MS8607 PHT sensor.
 */
class Adafruit_MS8607 {
public:
  Adafruit_MS8607(void);
  ~Adafruit_MS8607(void);

  bool begin(TwoWire *wire = &Wire, int32_t sensor_id = 0);
  bool init(int32_t sensor_id);

  bool reset(void);

  bool getEvent(sensors_event_t *pressure, sensors_event_t *temp,
                sensors_event_t *humidity);
  Adafruit_Sensor *getTemperatureSensor(void);
  Adafruit_Sensor *getPressureSensor(void);
  Adafruit_Sensor *getHumiditySensor(void);

  float _pressure,  ///< The current pressure measurement
      _temperature, ///< the current temperature measurement
      _humidity;    ///< The current humidity measurement

protected:
  // uint16_t _sensorid_presure;     ///< ID number for pressure
  uint16_t _sensorid_temp;     ///< ID number for temperature
  uint16_t _sensorid_pressure; ///< ID number for pressure
  uint16_t _sensorid_humidity; ///< ID number for humidity

  Adafruit_I2CDevice *pt_i2c_dev = NULL; ///< Pointer to I2C bus interface for
                                         ///< the pressure & temperature sensor
  Adafruit_I2CDevice *hum_i2c_dev =
      NULL; ///< Pointer to I2C bus interface for the humidity sensor

  Adafruit_MS8607_Temp *temp_sensor = NULL; ///< Temp sensor data object
  Adafruit_MS8607_Pressure *pressure_sensor =
      NULL; ///< Pressure sensor data object
  Adafruit_MS8607_Humidity *humidity_sensor =
      NULL; ///< Humidity sensor data object

private:
  bool _read(void);
  bool _read_humidity(void);
  bool _psensor_crc_check(uint16_t *n_prom, uint8_t crc);
  bool _hsensor_crc_check(uint16_t value, uint8_t crc);

  void _fetchTempCalibrationValues(void);
  void _fetchHumidityCalibrationValues(void);

  friend class Adafruit_MS8607_Temp;     ///< Gives access to private members to
                                         ///< Temperature data object
  friend class Adafruit_MS8607_Pressure; ///< Gives access to private members to
                                         ///< Pressure data object
  friend class Adafruit_MS8607_Humidity; ///< Gives access to private members to
                                         ///< Humidity data object

  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillPressureEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillHumidityEvent(sensors_event_t *humidity, uint32_t timestamp);

  void _applyTemperatureCorrection(void);
  uint8_t psensor_resolution_osr;
  uint16_t press_sens, press_offset, press_sens_temp_coeff,
      press_offset_temp_coeff, ref_temp,
      temp_temp_coeff; ///< calibration constants
  ms8607_hum_clock_stretch_t
      _hum_sensor_i2c_read_mode; ///< The current I2C mode to use for humidity
                                 ///< reads
};
#endif
