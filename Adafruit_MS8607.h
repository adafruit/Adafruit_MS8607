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
} ms8607_pressure_range_t;

class Adafruit_MS8607;

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
 * Driver for the Adafruit MS8607 PHT sensor.
 */
class Adafruit_MS8607 {
public:
  Adafruit_MS8607(void);
  ~Adafruit_MS8607(void);

  bool begin(TwoWire *wire = &Wire, int32_t sensor_id = 0);
  bool _init(int32_t sensor_id);

  bool reset(void);
  // SENSOR
  bool getEvent(sensors_event_t *pressure, sensors_event_t *temp,
                sensors_event_t *humidity);
  Adafruit_Sensor *getTemperatureSensor(void);
  Adafruit_Sensor *getPressureSensor(void);

  bool _read(void);
  float _pressure,  ///< The current pressure measurement
      _temperature, ///< the current temperature measurement
      _humidity;    ///< The current humidity measurement

protected:
  // uint16_t _sensorid_presure;     ///< ID number for pressure
  uint16_t _sensorid_temp;     ///< ID number for temperature
  uint16_t _sensorid_pressure; ///< ID number for pressure

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

  Adafruit_MS8607_Temp *temp_sensor = NULL; ///< Temp sensor data object
  Adafruit_MS8607_Pressure *pressure_sensor =
      NULL; ///< Pressure sensor data object

private:
  bool _psensor_crc_check(uint16_t *n_prom, uint8_t crc);
  void _fetchTempCalibrationValues(void);
  void _fetchHumidityCalibrationValues(void);

  friend class Adafruit_MS8607_Temp;     ///< Gives access to private members to
                                         ///< Temperature data object
  friend class Adafruit_MS8607_Pressure; ///< Gives access to private members to
                                         ///< Pressure data object

  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillPressureEvent(sensors_event_t *temp, uint32_t timestamp);

  void _applyTemperatureCorrection(void);
  void _applyHumidityCorrection(void);
  uint8_t psensor_resolution_osr;
  uint16_t press_sens, press_offset, press_sens_temp_coeff,
      press_offset_temp_coeff, ref_temp,
      temp_temp_coeff; ///< calibration constants
};
#endif
