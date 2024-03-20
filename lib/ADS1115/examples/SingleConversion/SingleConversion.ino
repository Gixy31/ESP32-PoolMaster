#include <Arduino.h>
#include <Wire.h>
#include <ADS1115.h>

// Single converters, either single ended of differential
// Manual for library: http://lygte-info.dk/project/ADS1115Library%20UK.html
// By HKJ from lygte-info.dk


ADS1115 adc;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  adc.setSpeed(ADS1115_SPEED_16SPS);
}

void loop() {

  Serial.print("Value on input #0 is: ");
  Serial.println(adc.convert(ADS1115_CHANNEL0, ADS1115_RANGE_6144));

  Serial.print("Value between input #2 and #3 is: ");
  Serial.println(adc.convert(ADS1115_CHANNEL23, ADS1115_RANGE_6144));

  Serial.println();

  delay(1000);
}