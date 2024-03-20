
#include <Arduino.h>
#include "ADS1115.h"


#include <Wire.h>
#include <limits.h>

// Manual for library: http://lygte-info.dk/project/ADS1115Library%20UK.html
// This is version 1.01 from 2020-6-19
// By HKJ from lygte-info.dk

//--------------------------------------------------------------------------------
// Write a 16 bit register on ADS1115
void ADS1115::i2cWriteReg(byte reg, unsigned int value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((byte) (value >> 8));
  Wire.write((byte) (value & 0xff));
  Wire.endTransmission();
}

//--------------------------------------------------------------------------------
// Write a 16 bit register from ADS1115
unsigned int ADS1115::i2cReadReg(byte reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte) 2);
  byte b1 = Wire.read();
  byte b2 = Wire.read();
  return (b1 << 8) | b2;
}

//--------------------------------------------------------------------------------
// Start a conversion, do not wait for result
void ADS1115::start(byte no, byte range) {
  this->no=no;
  unsigned int config = (1 << 15) | (no << 12) | (range << 9) | (1 << 8) | (speed << 5) | (0b11 << 0);
  i2cWriteReg(1, config);
}

//--------------------------------------------------------------------------------
// Read the conversion result from ADS1115, will wait if required
int ADS1115::read() {
  while ((i2cReadReg(1) & 0x8000) == 0) delay(1);
  return i2cReadReg(0);
}

//--------------------------------------------------------------------------------
// Convert a value and return it
int ADS1115::convert(byte no, byte range) {
  this->no=no;
  unsigned int config = (1 << 15) | (no << 12) | (range << 9) | (1 << 8) | (speed << 5) | (0b11 << 0);
  i2cWriteReg(1, config);
  while (!ready()) delay(1);
  return i2cReadReg(0);
}

//--------------------------------------------------------------------------------
// Convert a some values, first value is used to select optimum range. An average of the read values are returned.
float ADS1115::convertAutoScale(byte no, byte avg) {
  this->no=no;
  byte range = ADS1115_RANGE_6144;
  float mult = 6.144 / 0x7fff;
  long result = 0;
  if (avg==0) avg=1;
  result = convert(no, range);
  if (result < 1000) {
    range = ADS1115_RANGE_0256;
    mult = 0.256 / 0x7fff;
  } else if (result < 2000) {
    range = ADS1115_RANGE_0512;
    mult = 0.512 / 0x7fff;
  } else if (result < 4000) {
    range = ADS1115_RANGE_1024;
    mult = 1.024 / 0x7fff;
  } else if (result < 8000) {
    range= ADS1115_RANGE_2048;
    mult = 2.048 / 0x7fff;
  } else if (result < 16000) {
    range= ADS1115_RANGE_4096;
    mult = 4.096 / 0x7fff;
  } else if (avg==1) {   // We already have the value and only one measurement is required
    return result * mult;
  }
  result = 0;
  for (byte i = 0; i < avg; i++) {
    result += convert(no, range);
  }
  return result * mult / avg;
}

//--------------------------------------------------------------------------------
// Clear stored channel list and release sample memory
void ADS1115Scanner::clearChannels() {
  channels = 0;
  done = true;
  delete sampleMem;
  sampleMem = NULL;
}

//--------------------------------------------------------------------------------
// Add a channel definition to read list and release sample memory.
void ADS1115Scanner::addChannel(byte no, byte range) {
  if (channels >= maxChannels) return;
  channelRange[channels] = range;
  channelNo[channels] = no;
  channels++;
  delete sampleMem;
  sampleMem = NULL;
}

//--------------------------------------------------------------------------------
// Define number of samples and release sample memory
void ADS1115Scanner::setSamples(byte n) {
  samples = max(n, (byte) 1);
  done = true;
  delete sampleMem;
  sampleMem = NULL;
}

//--------------------------------------------------------------------------------
// Start conversions from the stored list
void ADS1115Scanner::start() {
  curNo = 0;
  curSample = 0;
  done = false;
  ADS1115::start(channelNo[curNo], channelRange[curNo]);
}

//--------------------------------------------------------------------------------
// Update conversion state, this will also allocate sample memory if required.
boolean ADS1115Scanner::update() {
  if (!ADS1115::ready()) return false;
  if (sampleMem == NULL) {
    sampleMem = new int[samples * channels];
  }
  sampleMem[sampleOffset(curNo,curSample)] = ADS1115::read();
  curNo++;
  if (curNo >= channels) {
    curSample++;
    curNo = 0;
    if (curSample >= samples) {
      done = true;
    }
  }
  if (!done) ADS1115::start(channelNo[curNo], channelRange[curNo]);
  return done;
}

//--------------------------------------------------------------------------------
// Return an average of read value, but discard highest and lowest value.
int ADS1115Scanner::readFilter(int no) {
  if (samples<=2) return 0;
  int mi = INT_MAX;
  int mx = INT_MIN;
  boolean miUsed = false;
  boolean mxUsed = false;
  for (int i = 0; i < samples; i++) {
    int v = sampleMem[sampleOffset(no,i)];
    mi = min(v, mi);
    mx = max(v, mx);
  }
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    int v = sampleMem[sampleOffset(no,i)];
    if (!miUsed && v == mi) {
      miUsed = true;
    } else if (!mxUsed && v == mx) {
      mxUsed = true;
    }
    else sum += v;
  }
  return sum / (samples - 2);
}

//--------------------------------------------------------------------------------
// Return an average of read value
int ADS1115Scanner::readAverage(int no) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += sampleMem[sampleOffset(no,i)];
  }
  return sum / samples;
}


//--------------------------------------------------------------------------------
int ADS1115ScaleInt::scale(int v) {
  return (int) (((long)(v - v1)) * ((long)(ref2 - ref1)) / (v2 - v1)) + ref1;
}

//--------------------------------------------------------------------------------
long ADS1115ScaleLong::scale(int v) {
  return (long) (((long long)(v - v1)) * ((long long)(ref2 - ref1)) / (v2 - v1)) + ref1;
}

//--------------------------------------------------------------------------------
float ADS1115ScaleFloat::scale(int v) {
  return ((v - v1) * (ref2 - ref1)) / (v2 - v1) + ref1;
}

