#include <Arduino.h>
#include <Wire.h>
#include <ADS1115.h>

// Auto scale conversion
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
  Serial.print(adc.convertAutoScale(ADS1115_CHANNEL0, 10), 5);
  Serial.println("V");

  Serial.print("Value between input #2 and #3 is: ");
  Serial.print(adc.convertAutoScale(ADS1115_CHANNEL23, 10), 5);
  Serial.println("V");

  Serial.println("V as average over 10 measurements");

  Serial.println();


  delay(1000);
}