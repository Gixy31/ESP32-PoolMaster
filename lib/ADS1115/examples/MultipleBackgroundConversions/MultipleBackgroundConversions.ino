#include <Arduino.h>
#include <Wire.h>
#include <ADS1115.h>

// Background conversion multiple inputs with average
// Manual for library: http://lygte-info.dk/project/ADS1115Library%20UK.html
// By HKJ from lygte-info.dk


ADS1115Scanner adc;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  adc.setSpeed(ADS1115_SPEED_16SPS);
  adc.addChannel(ADS1115_CHANNEL0, ADS1115_RANGE_6144);
  adc.addChannel(ADS1115_CHANNEL13, ADS1115_RANGE_6144);
  adc.setSamples(10);
  adc.start();
  Serial.println("Wait...");
}

long i = 0;

void loop() {
  i++;
  adc.update();
  if (adc.ready()) {
    Serial.print("Value on input #0 is: ");
    Serial.println(adc.readAverage(0));
    Serial.print("Value between input #1 and #3 is: ");
    Serial.print(adc.readAverage(1));
    Serial.println(" both calculated as average of 10 measurements.");
    Serial.print("While doing this the Arduino counted to: ");
    Serial.println(i);
    Serial.println();
    i = 0;
    adc.start();
  }
}