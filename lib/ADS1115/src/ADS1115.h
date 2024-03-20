#ifndef _ADS1151_H
#define _ADS1151_H

// Manual for library: http://lygte-info.dk/project/ADS1115Library%20UK.html
// This is version 1.01 from 2020-6-19
// By HKJ from lygte-info.dk

// This library include a couple of ways to use the ADS1115
// ADS1115 class:
//	convert			Read a single analog value
//	convertAutoScale 	Read a analog value, uses multiple readings to select range and average some samples
// 	start, ready, read	Make a reading while doing other stuff
// ADS1115Scanner class:	Will read multiple channes and samples, all samples are saved
//	addChannel		Define a input channel, all defined channels will be converter when start is called.
//	start			Start a batch of conversions
//	update			Must be called at regular interval and will switch to next converters when needed
//	ready			Check if all conversion are done
//	readAverage		Calculate average for last conversion and return it
//	readFilter		Remove highest and lowest value, calculate average for the other values and return it.

#include "Arduino.h"

#define ADS1115ADDRESS (0x48)
// Addrpin gnd:0x48, vdd:0x49, SDA:0x4A, SCL:0x4B

// Two digits means differential measurement
#define ADS1115_CHANNEL01 0b000
#define ADS1115_CHANNEL03 0b001
#define ADS1115_CHANNEL13 0b010
#define ADS1115_CHANNEL23 0b011
#define ADS1115_CHANNEL0 0b100
#define ADS1115_CHANNEL1 0b101
#define ADS1115_CHANNEL2 0b110
#define ADS1115_CHANNEL3 0b111

// Range in mV
#define ADS1115_RANGE_6144 0b000    // Maximum value is Vcc
#define ADS1115_RANGE_4096 0b001
#define ADS1115_RANGE_2048 0b010
#define ADS1115_RANGE_1024 0b011
#define ADS1115_RANGE_0512 0b100
#define ADS1115_RANGE_0256 0b101    // Maximum value is 256mV

// Sample speed, the lower speeds are good at filtering noise.
#define ADS1115_SPEED_8SPS 0
#define ADS1115_SPEED_16SPS 1
#define ADS1115_SPEED_32SPS 2
#define ADS1115_SPEED_64SPS 3
#define ADS1115_SPEED_128SPS 4
#define ADS1115_SPEED_250SPS 5
#define ADS1115_SPEED_475SPS 6
#define ADS1115_SPEED_860SPS 7


// Can read on ADC input, either directly or start a conversion where the result is read later.
class ADS1115 {
  private:
    byte addr;
    byte speed;
    byte no;
    void i2cWriteReg(byte reg, unsigned int value);
    unsigned int i2cReadReg(byte reg);

  public:
    ADS1115(byte addr) {
      this->addr = addr;
      speed = ADS1115_SPEED_64SPS;
    }
    ADS1115() {
      this->addr = ADS1115ADDRESS;
      speed = ADS1115_SPEED_64SPS;
    }

    unsigned int readReg(byte reg) {
      return i2cReadReg(reg);
    }

    void setSpeed(byte speed) {
      this->speed = speed;
    }

    // Do a full conversion
    int convert(byte no, byte range);

    // Do two or more conversions, the first conversion is used to decide scale.
    float convertAutoScale(byte no, byte avg = 1);

    // Start a conversion
    void start(byte no, byte range);

    // Check if conversion is done
    boolean ready() {
      return ((i2cReadReg(1) & 0x8000) != 0);
    }

    // Read the result from the started conversion, this routine will wait if necessary
    int read();

};

const byte maxChannels = 4;

// Read one or more input and can calculate average for multiple readings.
class ADS1115Scanner: public ADS1115 {
  private:
    byte channels = 0;
    byte samples = 0;
    byte channelRange[maxChannels];
    byte channelNo[maxChannels];
    byte curNo;
    byte curSample;
    int *sampleMem = NULL;
    boolean done = true;

    inline int sampleOffset(byte no, byte sample) {
      return no * samples + sample;
    }
  public:
    ADS1115Scanner() {};
    ADS1115Scanner(byte addr): ADS1115(addr) {};
    void clearChannels();

    // Define a channel
    void addChannel(byte no, byte range);

    // Define number of samples
    void setSamples(byte n);

    // Start a conversion for specificed number of channels and samples
    void start();

    // Must be called regulary for the routine to switch to next channel/sample
    boolean update();

    // Check if it is ready
    boolean ready() {
      return done;
    }

    // Get the average result for a channel but without the lowest and highest values.
    // This function is only valid with more than two samples.
    int readFilter(int no);

    // Get the average result for a channel, it will include lowest and highest value.
    int readAverage(int no);

    // Return a pointer to the samples for specified channel
    int *readSamples(int no) {
      return &sampleMem[sampleOffset(no, 0)];
    }
};


// Scale a ADC reading uses integer math with result and ref in long.
// Call with two reference points, for each point the ADC value (v1) and the desired value (ref) must be specified.
class ADS1115ScaleInt {
  public:
    int v1;
    int ref1;
    int v2;
    int ref2;
    ADS1115ScaleInt() {
      v1 = 0;
      ref1 = 0;
      v2 = 10000;
      ref2 = 10000;
    }
    ADS1115ScaleInt(int v1, int ref1, int v2, int ref2) {
      this->v1 = v1;
      this->ref1 = ref1;
      this->v2 = v2;
      this->ref2 = ref2;
    }
    void setRef(int v1, int ref1, int v2, int ref2) {
      this->v1 = v1;
      this->ref1 = ref1;
      this->v2 = v2;
      this->ref2 = ref2;
    }
    int scale(int v);

};


// Scale a ADC reading uses integer math with result and ref in long.
// Call with two reference points, for each point the ADC value (v1) and the desired value (ref) must be specified.
class ADS1115ScaleLong {
  public:
    int v1;
    long ref1;
    int v2;
    long ref2;
    ADS1115ScaleLong() {
      v1 = 0;
      ref1 = 0;
      v2 = 10000;
      ref2 = 10000;
    }
    ADS1115ScaleLong(int v1, long ref1, int v2, long ref2) {
      this->v1 = v1;
      this->ref1 = ref1;
      this->v2 = v2;
      this->ref2 = ref2;
    }
    void setRef(int v1, long ref1, int v2, long ref2) {
      this->v1 = v1;
      this->ref1 = ref1;
      this->v2 = v2;
      this->ref2 = ref2;
    }
    long scale(int v);

};

class ADS1115ScaleFloat {
  public:
    int v1;
    float ref1;
    int v2;
    float ref2;
    ADS1115ScaleFloat() {
      v1 = 0;
      ref1 = 0;
      v2 = 10000;
      ref2 = 10000;
    }
    ADS1115ScaleFloat(int v1, float ref1, int v2, float ref2) {
      this->v1 = v1;
      this->ref1 = ref1;
      this->v2 = v2;
      this->ref2 = ref2;
    }
    void setRef(int v1, float ref1, int v2, float ref2) {
      this->v1 = v1;
      this->ref1 = ref1;
      this->v2 = v2;
      this->ref2 = ref2;
    }
    float scale(int v);

};


#endif
