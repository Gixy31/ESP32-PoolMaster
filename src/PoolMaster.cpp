// Supervisory task

#include <Arduino.h>                // Arduino framework
#include "Config.h"
#include "PoolMaster.h"

// Functions prototypes
void ProcessCommand(char*);
void StartTime(void);
void readLocalTime(void);
bool saveParam(const char*,uint8_t );
bool saveParam(const char*,bool );
bool saveParam(const char*,unsigned long );
bool saveParam(const char*,double );
void SetPhPID(bool);
void SetOrpPID(bool);
void mqttErrorPublish(const char*);
void UpdateTFT(void);
void stack_mon(UBaseType_t&);

void PoolMaster(void *pvParameters)
{

                                                                                                                                                                                                 bool DoneForTheDay = false;                     // Reset actions done once per day
  bool d_calc = false;                            // Filtration duration computed
  bool cleaning_done = false;                     // daily cleaning done   

  static UBaseType_t hwm=0;                       // free stack size

  while(!startTasks);
  vTaskDelay(DT3);                                // Scheduling offset 

  esp_task_wdt_add(nullptr);
  TickType_t period = PT3;  
  TickType_t ticktime = xTaskGetTickCount(); 

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  {  
    // reset watchdog
    esp_task_wdt_reset();

    #ifdef CHRONO
    td = millis();
    #endif    

    // Handle OTA update
    ArduinoOTA.handle();

    //update pumps
    FiltrationPump.loop();
    PhPump.loop();
    ChlPump.loop();
    RobotPump.loop();  

    //reset time counters at midnight and send sync request to time server
    if (hour() == 0 && !DoneForTheDay)
    {
        //First store current Chl and Acid consumptions of the day in Eeprom
        storage.AcidFill = PhPump.GetTankFill();
        storage.ChlFill = ChlPump.GetTankFill();
        saveParam("AcidFill", storage.AcidFill);
        saveParam("ChlFill", storage.ChlFill);

        FiltrationPump.ResetUpTime();
        PhPump.ResetUpTime();
        PhPump.SetTankFill(storage.AcidFill);
        ChlPump.ResetUpTime();
        ChlPump.SetTankFill(storage.ChlFill);
        RobotPump.ResetUpTime();

        EmergencyStopFiltPump = false;
        d_calc = false;
        DoneForTheDay = true;
        cleaning_done = false;

        readLocalTime();
        setTime(timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec,timeinfo.tm_mday,timeinfo.tm_mon+1,timeinfo.tm_year-100);

    }
    else if(hour() == 1)
    {
        DoneForTheDay = false;
    }

    // Compute next Filtering duration and start/stop hours at 15:00 (to filter during the hotest period of the day)
    // Depending on water temperature, the filtration duration is either 2 hours, temp/3 or temp/2 hours.
    #ifdef DEBUG
    if (second() == 0 && !d_calc)
    #else
    if (hour() == 15 && !d_calc)
    #endif
    {
        if (storage.TempValue < storage.WaterTempLowThreshold){
            storage.FiltrationDuration = 2;}
        else if (storage.TempValue >= storage.WaterTempLowThreshold && storage.TempValue < storage.WaterTemp_SetPoint){
            storage.FiltrationDuration = round(storage.TempValue / 3.);}
        else if (storage.TempValue >= storage.WaterTemp_SetPoint){
            storage.FiltrationDuration = round(storage.TempValue / 2.);}
    
        storage.FiltrationStart = 15 - (int)round(storage.FiltrationDuration / 2.);
        if (storage.FiltrationStart < storage.FiltrationStartMin)
        storage.FiltrationStart = storage.FiltrationStartMin;    
        storage.FiltrationStop = storage.FiltrationStart + storage.FiltrationDuration;
        if (storage.FiltrationStop > storage.FiltrationStopMax)
        storage.FiltrationStop = storage.FiltrationStopMax;

        saveParam("FiltrStart",storage.FiltrationStart);  
        saveParam("FiltrStop",storage.FiltrationStop);  

        Debug.print(DBG_INFO,"Filtration duration: %dh",storage.FiltrationDuration);
        Debug.print(DBG_INFO,"Start: %dh - Stop: %dh",storage.FiltrationStart,storage.FiltrationStop);

        d_calc = true;
    }
    #ifdef DEBUG
    if(second() == 30 && d_calc) d_calc = false;
    #endif

    //start filtration pump as scheduled
    if (!EmergencyStopFiltPump && !FiltrationPump.IsRunning() && storage.AutoMode &&
        !PSIError && hour() >= storage.FiltrationStart && hour() < storage.FiltrationStop )
        FiltrationPump.Start();

    //start cleaning robot for 2 hours 30mn after filtration start
    if (FiltrationPump.IsRunning() && storage.AutoMode && !storage.WinterMode && !RobotPump.IsRunning() &&
        ((millis() - FiltrationPump.LastStartTime) / 1000 / 60) >= 30 && !cleaning_done)
    {
        RobotPump.Start();
        Debug.print(DBG_INFO,"Robot Start 30mn after Filtration");    
    }
    if(RobotPump.IsRunning() && storage.AutoMode && ((millis() - RobotPump.LastStartTime) / 1000 / 60) >= 120)
    {
        RobotPump.Stop();
        cleaning_done = true;
        Debug.print(DBG_INFO,"Robot Stop after: %d mn",(int)(millis()-RobotPump.LastStartTime)/1000/60);
    }

    // start PIDs with delay after FiltrationStart in order to let the readings stabilize
    // start inhibited if water temperature below threshold and/or in winter mode
    if (FiltrationPump.IsRunning() && storage.AutoMode && !storage.WinterMode && !PhPID.GetMode() &&
        ((millis() - FiltrationPump.LastStartTime) / 1000 / 60 >= storage.DelayPIDs) &&
        (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop) &&
        storage.TempValue >= storage.WaterTempLowThreshold)
    {
        //Start PIDs
        SetPhPID(true);
        SetOrpPID(true);
    }

    //stop filtration pump and PIDs as scheduled unless we are in AntiFreeze mode
    if (storage.AutoMode && FiltrationPump.IsRunning() && !AntiFreezeFiltering && (hour() >= storage.FiltrationStop || hour() < storage.FiltrationStart))
    {
        SetPhPID(false);
        SetOrpPID(false);
        FiltrationPump.Stop();
    }

    //Outside regular filtration hours, start filtration in case of cold Air temperatures (<-2.0deg)
    if (!EmergencyStopFiltPump && storage.AutoMode && !PSIError && !FiltrationPump.IsRunning() && ((hour() < storage.FiltrationStart) || (hour() > storage.FiltrationStop)) && (storage.TempExternal < -2.0))
    {
        FiltrationPump.Start();
        AntiFreezeFiltering = true;
    }

    //Outside regular filtration hours and if in AntiFreezeFiltering mode but Air temperature rose back above 2.0deg, stop filtration
    if (storage.AutoMode && FiltrationPump.IsRunning() && ((hour() < storage.FiltrationStart) || (hour() > storage.FiltrationStop)) && AntiFreezeFiltering && (storage.TempExternal > 2.0))
    {
        FiltrationPump.Stop();
        AntiFreezeFiltering = false;
    }

    //If filtration pump has been running for over 45secs but pressure is still low, stop the filtration pump, something is wrong, set error flag
    if (FiltrationPump.IsRunning() && ((millis() - FiltrationPump.LastStartTime) > 45000) && (storage.PSIValue < storage.PSI_MedThreshold))
    {
        FiltrationPump.Stop();
        PSIError = true;
        mqttErrorPublish("{\"PSI Error\":1}");
    }  

    // Over-pressure error
    if (storage.PSIValue > storage.PSI_HighThreshold)
    {
        FiltrationPump.Stop();
        PSIError = true;
        mqttErrorPublish("{\"PSI Error\":1}");
    } else if(storage.PSIValue >= storage.PSI_MedThreshold)
        PSIError = false;

    //UPdate Nextion TFT
    UpdateTFT();

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[PoolMaster] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif 

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }
}

//Enable/Disable Chl PID
void SetPhPID(bool Enable)
{
  if (Enable)
  {
    //Start PhPID
    PhPump.ClearErrors();
    storage.PhPIDOutput = 0.0;
    storage.PhPIDwindowStartTime = millis();
    PhPID.SetMode(AUTOMATIC);
    storage.Ph_RegulationOnOff = 1;
  }
  else
  {
    //Stop PhPID
    PhPID.SetMode(MANUAL);
    storage.Ph_RegulationOnOff = 0;
    storage.PhPIDOutput = 0.0;
    PhPump.Stop();
  }
}

//Enable/Disable Orp PID
void SetOrpPID(bool Enable)
{
  if (Enable)
  {
    //Start OrpPID
    ChlPump.ClearErrors();
    storage.OrpPIDOutput = 0.0;
    storage.OrpPIDwindowStartTime = millis();
    OrpPID.SetMode(AUTOMATIC);
    storage.Orp_RegulationOnOff = 1;

  }
  else
  {
    //Stop OrpPID
    OrpPID.SetMode(MANUAL);
    storage.Orp_RegulationOnOff = 0;
    storage.OrpPIDOutput = 0.0;
    ChlPump.Stop();
  }
}