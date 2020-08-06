// Basic demo for reading Humidity and Temperature
#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>

Adafruit_MS8607 ms8607;
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MS8607 test!");

  // Try to initialize!
  if (!ms8607.begin()) {
    Serial.println("Failed to find MS8607 chip");
    while (1) { delay(10); }
  }
  Serial.println("MS8607 Found!\n\n\n");

// //  ms8607.setDataRate(MS8607_RATE_1_HZ);
//   Serial.print("Data rate set to: ");
//   switch (ms8607.getDataRate()) {
//    case MS8607_RATE_ONE_SHOT: Serial.println("One Shot"); break;
//    case MS8607_RATE_1_HZ: Serial.println("1 Hz"); break;
//    case MS8607_RATE_7_HZ: Serial.println("7 Hz"); break;
//    case MS8607_RATE_12_5_HZ: Serial.println("12.5 Hz"); break;
//   }

}

void loop() {
    sensors_event_t temp, pressure, humidity;
    ms8607.getEvent(&pressure, &temp, &humidity);
    Serial.print("Temperature: ");Serial.print(temp.temperature); Serial.println(" degrees C");
    Serial.print("Pressure: ");Serial.print(ms8607._pressure); Serial.println(" hPa");
//   sensors_event_t temp;
//   sensors_event_t humidity;
//   ms8607.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
//   Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
//   Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  delay(500);
}
