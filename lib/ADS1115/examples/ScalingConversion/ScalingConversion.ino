#include <Arduino.h>
#include <Wire.h>
#include <ADS1115.h>

// Scaling a value
// Manual for library: http://lygte-info.dk/project/ADS1115Library%20UK.html
// By HKJ from lygte-info.dk


ADS1115 adc(ADS1115ADDRESS);
ADS1115ScaleFloat scale0;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  adc.setSpeed(ADS1115_SPEED_16SPS);

  // A 0 count reading is 0 and a 32767 reading is 204.8
  // This is the scale for a LM35 temperature sensor
  // 32767 is value from ADS1115 with 2.048 volt input, this is equivalent to 204.8°C on LM35 (theoretically).
  scale0.setRef(0, 0, 32676, 204.8);

  // Using two diodes, a resistor and differential input it would be possible to read negative temperatures.
  // The connection of diodes and resistor is shown in the LM35 datasheet.
}

void loop() {

  Serial.print("Value on input #0 is: ");
  int v = adc.convert(ADS1115_CHANNEL0, ADS1115_RANGE_2048);
  Serial.print(v);
  Serial.print("  this scales to (LM35 temperature): ");
  Serial.println(scale0.scale(v));

  delay(1000);
}