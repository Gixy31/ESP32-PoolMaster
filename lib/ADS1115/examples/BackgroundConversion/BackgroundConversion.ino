#include <Arduino.h>
#include <Wire.h>
#include <ADS1115.h>

// Background conversion
// Manual for library: http://lygte-info.dk/project/ADS1115Library%20UK.html
// By HKJ from lygte-info.dk


ADS1115 adc;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  adc.setSpeed(ADS1115_SPEED_8SPS);
  adc.start(ADS1115_CHANNEL0, ADS1115_RANGE_6144);
}

long i = 0;

void loop() {
  i++;
  if (adc.ready()) {
    Serial.print("Value on input #0 is: ");
    Serial.print(adc.read());
    Serial.print("  counted to ");
    Serial.print(i);
    Serial.println(" during the conversion");
    i = 0;
    adc.start(ADS1115_CHANNEL0, ADS1115_RANGE_6144);
  }  
}