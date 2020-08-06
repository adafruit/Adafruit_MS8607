// Basic demo for reading Humidity and Temperature
#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>

Adafruit_MS8607 ms8607;
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MS8607 unified sensor test!");

  // Try to initialize!
  if (!ms8607.begin()) {
    Serial.println("Failed to find MS8607 chip");
    while (1) { delay(10); }
  }
  Serial.println("MS8607 Found!\n\n\n");

}

void loop() {
    sensors_event_t temp, pressure, humidity;
    Adafruit_Sensor *pressure_sensor = ms8607.getPressureSensor();
    Adafruit_Sensor *temp_sensor = ms8607.getTemperatureSensor();
    Adafruit_Sensor *humidity_sensor = ms8607.getHumiditySensor();

    temp_sensor->getEvent(&temp);
    pressure_sensor->getEvent(&pressure);
    humidity_sensor->getEvent(&humidity);
    Serial.print("Temperature: ");Serial.print(temp.temperature); Serial.println(" degrees C");
    Serial.print("Pressure: ");Serial.print(pressure.pressure); Serial.println(" hPa");
    Serial.print("Relative Humidity: ");Serial.print(humidity.relative_humidity); Serial.println(" %rH");


  delay(500);
}
