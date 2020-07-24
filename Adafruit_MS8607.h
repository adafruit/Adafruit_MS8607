/*!
 *  @file Adafruit_MS8607.h
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
  (0x76) /**< The default pressure and temperature I2C address for the sensor. \
          */
#define MS8607_RH_ADDRESS (0x40) /**< Humidity I2C address for the sensor. */

//  Forward declarations of Wire and SPI for board/variant combinations that
//  don't have a default 'Wire' or 'SPI'
extern TwoWire Wire; /**< Forward declaration of Wire object */
extern SPIClass SPI; /**< Forward declaration of SPI object */

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

/*!

class Adafruit_MS8607;

// /** Adafruit Unified Sensor interface for temperature component of MS8607 */
// class Adafruit_MS8607_PT : public Adafruit_Sensor {
// public:
//   /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
//       @param parent A pointer to the MS8607 class */
//   Adafruit_MS8607_PT(Adafruit_MS8607 *parent) { _theMS8607 = parent; }
//   bool getEvent(sensors_event_t *);
//   void getSensor(sensor_t *);

// private:
//   int _sensorID = XXXX;
//   Adafruit_MS8607 *_theMS8607 = NULL;
// };

// /** Adafruit Unified Sensor interface for pressure component of MS8607 */
// class Adafruit_MS8607_Pressure : public Adafruit_Sensor {
// public:
//   /** @brief Create an Adafruit_Sensor compatible object for the PHT
//       @param parent A pointer to the MS8607 class */
//   Adafruit_MS8607_PHT(Adafruit_MS8607 *parent) { _theMS8607 = parent; }
//   bool getEvent(sensors_event_t *);
//   void getSensor(sensor_t *);

// private:
//   int _sensorID = 0;
//   Adafruit_MS8607 *_theMS8607 = NULL;
// };

/**
 * Driver for the Adafruit MS8607 PHT sensor.
 */
class Adafruit_MS8607 {
public:
  Adafruit_MS8607();
  ~Adafruit_MS8607();

  bool begin(uint8_t i2c_addr = MS8607_PT_ADDRESS, TwoWire *wire = &Wire,
             int32_t sensor_id = 0);
  bool _init(int32_t sensor_id);
  // ms8607_rate_t getDataRate(void);
  // void setDataRate(ms8607_rate_t data_rate);

  bool reset(void);

  // bool getEvent(sensors_event_t *humidity, sensors_event_t *temp);
  // Adafruit_Sensor *getTemperatureSensor(void);
  // Adafruit_Sensor *getHumiditySensor(void);

protected:
  // bool _read(void);
  // virtual bool _init(int32_t sensor_id);

  // float corrected_temp,   ///< Last reading's temperature (C) before scaling
  //     corrected_humidity; ///< Last reading's humidity (percent) before
  //     scaling

  uint16_t _sensorid_humidity; ///< ID number for humidity
  uint16_t _sensorid_temp;     ///< ID number for temperature

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to I2C bus interface

  //  Adafruit_MS8607_Humidity *rh_sensor = NULL;
  //   Adafruit_MS8607_PT *pt_sensor = NULL;

private:
  bool _psensor_crc_check(uint16_t *n_prom, uint8_t crc);
  void _fetchTempCalibrationValues(void);
  void _fetchHumidityCalibrationValues(void);
  friend class Adafruit_MS8607_Temp;     ///< Gives access to private members to
                                         ///< Temp data object
  friend class Adafruit_MS8607_Humidity; ///< Gives access to private members to
                                         ///< Humidity data object

  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillHumidityEvent(sensors_event_t *humidity, uint32_t timestamp);

  void _applyTemperatureCorrection(void);
  void _applyHumidityCorrection(void);
};

#endif

//   Adafruit_Sensor *getPTSensor(void);
//   Adafruit_Sensor *getRHSensor(void);

// private:
//   TwoWire *_wire; /**< Wire object */
//   SPIClass *_spi; /**< SPI object */

//   Adafruit_MS8607_Humidity *rh_sensor = NULL;
//   Adafruit_MS8607_PT *pt_sensor = NULL;

//   uint8_t _i2caddr;

//   int32_t _sensorID;
//   int32_t t_fine;
//   int8_t _cs, _mosi, _miso, _sck;
//   ms8607_calib_data _ms8607_calib;
//   config _configReg;
//   ctrl_meas _measReg;
// };

// #endif
