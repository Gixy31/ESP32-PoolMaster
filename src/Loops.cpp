#include <Arduino.h>                // Arduino framework
#include "Config.h"
#include "PoolMaster.h"

// Setup oneWire instances to communicate with temperature sensors (one bus per sensor)
static OneWire oneWire_W(ONE_WIRE_BUS_W);
static OneWire oneWire_A(ONE_WIRE_BUS_A);
// Pass our oneWire reference to Dallas Temperature library instance
static DallasTemperature sensors_W(&oneWire_W);
static DallasTemperature sensors_A(&oneWire_A);
// MAC Addresses of DS18b20 water & Air temperature sensor
static DeviceAddress DS18B20_W = { 0x28, 0x9F, 0x24, 0x24, 0x0C, 0x00, 0x00, 0xA9 };
static DeviceAddress DS18B20_A = { 0x28, 0xB0, 0x70, 0x75, 0xD0, 0x01, 0x3C, 0x9D };

// Setup an ADS1115 instance for analog measurements
static ADS1115Scanner adc(0x48);  // Address 0x48 is the default
static float ph_sensor_value;     // pH sensor current value
static float orp_sensor_value;    // ORP sensor current value
static float psi_sensor_value;    // PSI sensor current value

// Signal filtering library sample buffers
static RunningMedian samples_WTemp = RunningMedian(11);
static RunningMedian samples_ATemp = RunningMedian(11);
static RunningMedian samples_Ph    = RunningMedian(11);
static RunningMedian samples_Orp   = RunningMedian(11);
static RunningMedian samples_PSI   = RunningMedian(11);

void stack_mon(UBaseType_t&);
void lockI2C();
void unlockI2C();

//Update loop for ADS1115 measurements
// Some explanations: The sampling rate is set to 16sps in order to be sure that 
// every 125ms (which is the period of the task) there is a sample available. As it takes 3ms to
// update and restart the ADC, the whole loop takes a minimum of 3 + 1/SPS ms. If SPS was set to 
// 8sps, that means 128ms instead of 125ms. With 16sps, we have 3 + 62.5 < 125ms which is OK.
// With those settings, we get a value for each channel roughly every second: 9 values asked 
// (3 per channel), with height values retrieved per second -> 0,89 value per second. The value
// returned for each channel is the median of the three samples. Then, among the last 
// 11 samples returned, we take the 5 median ones and compute the mean as consolidated value.

void AnalogInit()
{
  adc.setSpeed(ADS1115_SPEED_16SPS);
  adc.addChannel(ADS1115_CHANNEL0, ADS1115_RANGE_6144);
  adc.addChannel(ADS1115_CHANNEL1, ADS1115_RANGE_6144);
  adc.addChannel(ADS1115_CHANNEL2, ADS1115_RANGE_6144);
  adc.setSamples(3);
}

void AnalogPoll(void *pvParameters)
{
  while (!startTasks) ;

  TickType_t period = PT1;  
  TickType_t ticktime = xTaskGetTickCount(); 
  static UBaseType_t hwm=0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  lockI2C();
  adc.start();
  unlockI2C();
  vTaskDelayUntil(&ticktime,period);
  
  for(;;)
  {
    #ifdef CHRONO
    td = millis();
    #endif

    lockI2C();
    adc.update();

    if(adc.ready()){                              // all conversions done ?
        orp_sensor_value = adc.readFilter(0) ;    // ORP sensor current value
        ph_sensor_value  = adc.readFilter(1) ;    // pH sensor current value
        psi_sensor_value = adc.readFilter(2) ;    // psi sensor current value
        adc.start();  
        
        //Ph
        samples_Ph.add(ph_sensor_value);          // compute average of pH from center 5 measurements among 11
        storage.PhValue = (samples_Ph.getAverage(5)*0.1875/1000.)*storage.pHCalibCoeffs0 + storage.pHCalibCoeffs1;

#ifdef SIMU
        if(!init_simu){
            if(newpHOutput) {
                pHTab[iw] = storage.PhPIDOutput;
                pHCumul = pHTab[0]+pHTab[1]+pHTab[2];
                iw++;
                iw %= 3;
            }
            storage.PhValue = pHLastValue + pHCumul/4500000.*(double)((millis()-pHLastTime)/3600000.);
            pHLastValue = storage.PhValue;
            pHLastTime = millis();
        } else {
            init_simu = false;
            pHLastTime = millis();
            pHLastValue = 7.0;
            storage.PhValue = pHLastValue;
            storage.OrpValue = OrpLastValue;
            OrpLastTime = millis();
            OrpLastValue = 730.0;
            for(uint8_t i=0;i<3;i++) {
                pHTab[i] = 0.;
                ChlTab[i] = 0.;
            }  
        }  
#endif

        //ORP
        samples_Orp.add(orp_sensor_value);                                                                    // compute average of ORP from last 5 measurements
        storage.OrpValue = (samples_Orp.getAverage(5)*0.1875/1000.)*storage.OrpCalibCoeffs0 + storage.OrpCalibCoeffs1;

#ifdef SIMU
        if(!init_simu){
            if(newChlOutput) {
            ChlTab[jw] = storage.OrpPIDOutput;
            ChlCumul = ChlTab[0]+ChlTab[1]+ChlTab[2];
            jw++;
            jw %= 3;
            }    
            storage.OrpValue = OrpLastValue + ChlCumul/36000.*(double)((millis()-OrpLastTime)/3600000.);
            OrpLastValue = storage.OrpValue;
            OrpLastTime = millis();    
        } 
#endif

        //PSI (water pressure)
        samples_PSI.add(psi_sensor_value);                                                                    // compute average of PSI from last 5 measurements
        storage.PSIValue = (samples_PSI.getAverage(5)*0.1875/1000.)*storage.PSICalibCoeffs0 + storage.PSICalibCoeffs1;

        Debug.print(DBG_DEBUG,"pH: %5.0f - %4.2f - ORP: %5.0f - %3.0fmV - PSI: %5.0f - %4.2fBar\r",
            ph_sensor_value,storage.PhValue,orp_sensor_value,storage.OrpValue,psi_sensor_value,storage.PSIValue);
    }
    unlockI2C();

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[AnalogPoll] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif 

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }  
}

void StatusLights(void *pvParameters)
{
  static uint8_t line = 0;
  uint8_t status;

  while (!startTasks) ;
  vTaskDelay(DT7);                                // Scheduling offset 

  TickType_t period = PT7;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  {
    #ifdef CHRONO
    td = millis();
    #endif 

    status = 0;
    status |= (line & 1) << 1;
    if(line == 0)
    {
        line = 1;
        status |= (storage.AutoMode & 1) << 2;
        status |= (AntiFreezeFiltering & 1) << 3;
        status |= (PSIError & 1) << 7;
    } else
    {
        line = 0;
        status |= (PhPID.GetMode() & 1) << 2;
        status |= (OrpPID.GetMode() & 1) << 3;
        status |= (!PhPump.TankLevel() & 1) << 4;
        status |= (!ChlPump.TankLevel() & 1) << 5;
        status |= (PhPump.UpTimeError & 1) << 6;
        status |= (ChlPump.UpTimeError & 1) << 7;  
    }
    (status & 0xF0) ? digitalWrite(BUZZER,HIGH) : digitalWrite(BUZZER,LOW) ;
    if(WiFi.status() == WL_CONNECTED) status |= 0x01;
        else status &= 0xFE;
    Debug.print(DBG_VERBOSE,"Status LED : 0x%02x",status);
    lockI2C();
    Wire.beginTransmission(0x38);
    Wire.write(~status);
    Wire.endTransmission();
    unlockI2C();

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[StatusLights] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }  
}

void pHRegulation(void *pvParameters)
{
  while (!startTasks) ;
  vTaskDelay(DT6);                                // Scheduling offset 

  TickType_t period = PT6;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  {
    #ifdef CHRONO
    td = millis();
    #endif 

    //do not compute PID if filtration pump is not running
    // Set also a lower limit at 30s (a lower pump duration does'nt mean anything)

    if (FiltrationPump.IsRunning()  && (PhPID.GetMode() == AUTOMATIC))
    {  
      if(PhPID.Compute()){
        Debug.print(DBG_VERBOSE,"Ph  regulation: %10.2f, %13.9f, %13.9f, %17.9f",storage.PhPIDOutput,storage.PhValue,storage.Ph_SetPoint,storage.Ph_Kp);
        if(storage.PhPIDOutput < (double)30000.) storage.PhPIDOutput = 0.;
        Debug.print(DBG_INFO,"Ph  regulation: %10.2f",storage.PhPIDOutput);
    #ifdef SIMU
        newpHOutput = true;
    #endif            
      }
    #ifdef SIMU
      else newpHOutput = false;
    #endif    
      /************************************************
       turn the Acid pump on/off based on pid output
      ************************************************/
      if (millis() - storage.PhPIDwindowStartTime > storage.PhPIDWindowSize)
      {
        //time to shift the Relay Window
        storage.PhPIDwindowStartTime += storage.PhPIDWindowSize;
      }
      if (storage.PhPIDOutput < millis() - storage.PhPIDwindowStartTime)
        PhPump.Stop();
      else
        PhPump.Start();   
    }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[pHRegulation] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }  
}

//Orp regulation loop
void OrpRegulation(void *pvParameters)
{
  while (!startTasks) ;
  vTaskDelay(DT5);                                // Scheduling offset 

  TickType_t period = PT5;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  { 
    #ifdef CHRONO
    td = millis();
    #endif 

    //do not compute PID if filtration pump is not running
    // Set also a lower limit at 30s (a lower pump duration does'nt mean anything)

    if (FiltrationPump.IsRunning() && (OrpPID.GetMode() == AUTOMATIC))
    {
      if(OrpPID.Compute()){
        Debug.print(DBG_VERBOSE,"ORP regulation: %10.2f, %13.9f, %12.9f, %17.9f",storage.OrpPIDOutput,storage.OrpValue,storage.Orp_SetPoint,storage.Orp_Kp);
        if(storage.OrpPIDOutput < (double)30000.) storage.OrpPIDOutput = 0.;    
          Debug.print(DBG_INFO,"Orp regulation: %10.2f",storage.OrpPIDOutput);
    #ifdef SIMU
          newChlOutput = true;
    #endif      
        }
    #ifdef SIMU
        else newChlOutput = false;
    #endif    
      /************************************************
       turn the Chl pump on/off based on pid output
      ************************************************/
      if (millis() - storage.OrpPIDwindowStartTime > storage.OrpPIDWindowSize)
      {
        //time to shift the Relay Window
        storage.OrpPIDwindowStartTime += storage.OrpPIDWindowSize;
      }
      if (storage.OrpPIDOutput < millis() - storage.OrpPIDwindowStartTime)
        ChlPump.Stop();
      else
        ChlPump.Start();
    }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[OrpRegulation] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);    
    vTaskDelayUntil(&ticktime,period);
  }
}

//Init DS18B20 one-wire library
void TempInit()
{
  bool error = false;
  // Start up the library
  sensors_W.begin();
  sensors_W.begin(); // two times to work-around of a OneWire library bug for enumeration
  sensors_A.begin();

  Debug.print(DBG_INFO,"1wire W devices: %d device(s) found",sensors_W.getDeviceCount());
  Debug.print(DBG_INFO,"1wire A devices: %d device(s) found",sensors_A.getDeviceCount());

  if (!sensors_W.getAddress(DS18B20_W, 0)) 
  {
    Debug.print(DBG_ERROR,"Unable to find address for bus W");
    error = true;
  }  
  else {
    Serial.printf("DS18B20_W: ");
    for(uint8_t i=0;i<8;i++){
      Serial.printf("%02x",DS18B20_W[i]);
      if(i<7) Serial.print(":");
        else Serial.printf("\r\n");
    }
  }  
  if (!sensors_A.getAddress(DS18B20_A, 0)) 
  {
    Debug.print(DBG_ERROR,"Unable to find address for bus A"); 
    error = true;
  }  
  else {
    Serial.printf("DS18B20_A: ");
    for(uint8_t i=0;i<8;i++){
      Serial.printf("%02x",DS18B20_A[i]);
      if(i<7) Serial.print(":");
        else Serial.printf("\r\n");
    }
  } 

  if(!error) 
  {
    // set the resolution
    sensors_W.setResolution(DS18B20_W, TEMPERATURE_RESOLUTION);
    sensors_A.setResolution(DS18B20_A, TEMPERATURE_RESOLUTION);

    //don't wait ! Asynchronous mode
    sensors_W.setWaitForConversion(false);
    sensors_A.setWaitForConversion(false);
  }
}

//Request temperature asynchronously
//in case of reading error, the buffer is not updated and the last value is kept
void getTemp(void *pvParameters)
{
  while (!startTasks) ;
  vTaskDelay(DT4);                                // Scheduling offset 

  TickType_t period = PT4;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  sensors_W.requestTemperatures();
  sensors_A.requestTemperatures();
  vTaskDelayUntil(&ticktime,period);
  
  for(;;)
  {        
    #ifdef CHRONO
    td = millis();
    #endif 

    double temp = sensors_W.getTempC(DS18B20_W);
    if (temp == NAN || temp == -127) {
      Debug.print(DBG_WARNING,"Error getting Water temperature");
    }  else storage.TempValue = temp;
    samples_WTemp.add(storage.TempValue);
    storage.TempValue = samples_WTemp.getAverage(5);
    Debug.print(DBG_DEBUG,"DS18B20_W: %6.2f°C",storage.TempValue);

    temp = sensors_A.getTempC(DS18B20_A);
    if (temp == NAN || temp == -127) {
      Debug.print(DBG_WARNING,"Error getting Air temperature");
    }  else storage.TempExternal = temp;
    samples_ATemp.add(storage.TempExternal);
    storage.TempExternal = samples_ATemp.getAverage(5);
    Debug.print(DBG_DEBUG,"DS18B20_A: %6.2f°C",storage.TempExternal);

    sensors_W.requestTemperatures();
    sensors_A.requestTemperatures();

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[getTemp] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }
}