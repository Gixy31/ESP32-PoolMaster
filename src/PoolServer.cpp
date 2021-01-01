// Function to process JSON commands received via MQTT.
// If the comand modify a setting parameter, it is saved in NVS and published back
// for other MQTT clients (dashboards)

#include <Arduino.h>
#include "config.h"
#include "PoolMaster.h"

// Functions prototypes
void mqttErrorPublish(const char*);
bool saveParam(const char*,uint8_t );
bool saveParam(const char*,bool );
bool saveParam(const char*,unsigned long );
bool saveParam(const char*,double );
void PublishSettings(void);
void simpLinReg(float * , float * , double & , double &, int );
void ProcessCommand(String );
void PublishDataCallback(Task*);
void setPublishPeriod(unsigned long);
void SetPhPID(bool);
void SetOrpPID(bool);


void ProcessCommand(String JSONCommand)
{
  //Json Document
  StaticJsonDocument<200> command;

  //Parse Json object and find which command it is
  DeserializationError error = deserializeJson(command,JSONCommand);

  // Test if parsing succeeds.
  if (error)
  {
    Serial << F("Json parseObject() failed");
    return;
  }
  else
  {
    Serial << F("Json parseObject() success - ") << endl;
    DEBUG_PRINT(JSONCommand);

    //Provide the external temperature. Should be updated regularly and will be used to start filtration for 10mins every hour when temperature is negative
    if (command.containsKey(F("TempExt")))
    {
      storage.TempExternal = command["TempExt"].as<float>();
      Serial << F("External Temperature: ") << storage.TempExternal << F("deg") << endl;
    }
    //"PhCalib" command which computes and sets the calibration coefficients of the pH sensor response based on a multi-point linear regression
    //{"PhCalib":[4.02,3.8,9.0,9.11]}  -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
    else if (command.containsKey(F("PhCalib")))
    {
      float CalibPoints[12];//Max six calibration point-couples! Should be plenty enough
      int NbPoints = (int)copyArray(command[F("PhCalib")].as<JsonArray>(),CalibPoints);        
      Serial << F("PhCalib command - ") << NbPoints << F(" points received: ");
      for (int i = 0; i < NbPoints; i += 2)
        Serial << CalibPoints[i] << F(",") << CalibPoints[i + 1] << F(" - ");
      Serial << _endl;

      if (NbPoints == 2) //Only one pair of points. Perform a simple offset calibration
      {
        Serial << F("2 points. Performing a simple offset calibration") << _endl;

        //compute offset correction
        storage.pHCalibCoeffs1 += CalibPoints[1] - CalibPoints[0];

        //Set slope back to default value
        storage.pHCalibCoeffs0 = 3.76;

        Serial << F("Calibration completed. Coeffs are: ") << storage.pHCalibCoeffs0 << F(",") << storage.pHCalibCoeffs1 << _endl;
      }
      else if ((NbPoints > 3) && (NbPoints % 2 == 0)) //we have at least 4 points as well as an even number of points. Perform a linear regression calibration
      {
        Serial << NbPoints / 2 << F(" points. Performing a linear regression calibration") << _endl;

        float xCalibPoints[NbPoints / 2];
        float yCalibPoints[NbPoints / 2];

        //generate array of x sensor values (in volts) and y rated buffer values
        //storage.PhValue = (storage.pHCalibCoeffs0 * ph_sensor_value) + storage.pHCalibCoeffs1;
        for (int i = 0; i < NbPoints; i += 2)
        {
          xCalibPoints[i / 2] = (CalibPoints[i] - storage.pHCalibCoeffs1) / storage.pHCalibCoeffs0;
          yCalibPoints[i / 2] = CalibPoints[i + 1];
        }

        //Compute linear regression coefficients
        simpLinReg(xCalibPoints, yCalibPoints, storage.pHCalibCoeffs0, storage.pHCalibCoeffs1, NbPoints / 2);

        Serial << F("Calibration completed. Coeffs are: ") << storage.pHCalibCoeffs0 << F(",") << storage.pHCalibCoeffs1 << _endl;
      }
      //Store the new coefficients in eeprom
      saveParam("pHCalibCoeffs0",storage.pHCalibCoeffs0);
      saveParam("pHCalibCoeffs1",storage.pHCalibCoeffs1);          
      PublishSettings();
    }
    //"OrpCalib" command which computes and sets the calibration coefficients of the Orp sensor response based on a multi-point linear regression
    //{"OrpCalib":[450,465,750,784]}   -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
    else if (command.containsKey(F("OrpCalib")))
    {
      float CalibPoints[12];//Max six calibration point-couples! Should be plenty enough
      int NbPoints = (int)copyArray(command[F("OrpCalib")].as<JsonArray>(),CalibPoints);
      Serial << F("OrpCalib command - ") << NbPoints << F(" points received: ");
      for (int i = 0; i < NbPoints; i += 2)
        Serial << CalibPoints[i] << F(",") << CalibPoints[i + 1] << F(" - ");
      Serial << _endl;

      if (NbPoints == 2) //Only one pair of points. Perform a simple offset calibration
      {
        Serial << F("2 points. Performing a simple offset calibration") << _endl;

        //compute offset correction
        storage.OrpCalibCoeffs1 += CalibPoints[1] - CalibPoints[0];

        //Set slope back to default value
        storage.OrpCalibCoeffs0 = -1000;

        Serial << F("Calibration completed. Coeffs are: ") << storage.OrpCalibCoeffs0 << F(",") << storage.OrpCalibCoeffs1 << _endl;
      }
      else if ((NbPoints > 3) && (NbPoints % 2 == 0)) //we have at least 4 points as well as an even number of points. Perform a linear regression calibration
      {
        Serial << NbPoints / 2 << F(" points. Performing a linear regression calibration") << _endl;

        float xCalibPoints[NbPoints / 2];
        float yCalibPoints[NbPoints / 2];

        //generate array of x sensor values (in volts) and y rated buffer values
        //storage.OrpValue = (storage.OrpCalibCoeffs0 * orp_sensor_value) + storage.OrpCalibCoeffs1;
        for (int i = 0; i < NbPoints; i += 2)
        {
          xCalibPoints[i / 2] = (CalibPoints[i] - storage.OrpCalibCoeffs1) / storage.OrpCalibCoeffs0;
          yCalibPoints[i / 2] = CalibPoints[i + 1];
        }

        //Compute linear regression coefficients
        simpLinReg(xCalibPoints, yCalibPoints, storage.OrpCalibCoeffs0, storage.OrpCalibCoeffs1, NbPoints / 2);

        Serial << F("Calibration completed. Coeffs are: ") << storage.OrpCalibCoeffs0 << F(",") << storage.OrpCalibCoeffs1 << _endl;
      }
      //Store the new coefficients in eeprom
      saveParam("OrpCalibCoeffs0",storage.OrpCalibCoeffs0);
      saveParam("OrpCalibCoeffs1",storage.OrpCalibCoeffs1);          
      PublishSettings();
    }
    //"PSICalib" command which computes and sets the calibration coefficients of the Electronic Pressure sensor response based on a linear regression and a reference mechanical sensor (typically located on the sand filter)
    //{"PSICalib":[0,0,0.71,0.6]}   -> multi-point linear regression calibration (minimum 2 point-couple, 6 max.) in the form [ElectronicPressureSensorReading_0, MechanicalPressureSensorReading_0, xx, xx, ElectronicPressureSensorReading_n, ElectronicPressureSensorReading_n]
    else if (command.containsKey(F("PSICalib")))
    {
      float CalibPoints[12];//Max six calibration point-couples! Should be plenty enough, typically use two point-couples (filtration ON and filtration OFF)
      int NbPoints = (int)copyArray(command[F("PSICalib")].as<JsonArray>(),CalibPoints);
      Serial << F("PSICalib command - ") << NbPoints << F(" points received: ");
      for (int i = 0; i < NbPoints; i += 2)
        Serial << CalibPoints[i] << F(",") << CalibPoints[i + 1] << F(" - ");
      Serial << _endl;

      if ((NbPoints > 3) && (NbPoints % 2 == 0)) //we have at least 4 points as well as an even number of points. Perform a linear regression calibration
      {
        Serial << NbPoints / 2 << F(" points. Performing a linear regression calibration") << _endl;

        float xCalibPoints[NbPoints / 2];
        float yCalibPoints[NbPoints / 2];

        //generate array of x sensor values (in volts) and y rated buffer values
        //storage.OrpValue = (storage.OrpCalibCoeffs0 * orp_sensor_value) + storage.OrpCalibCoeffs1;
        //storage.PSIValue = (storage.PSICalibCoeffs0 * psi_sensor_value) + storage.PSICalibCoeffs1;
        for (int i = 0; i < NbPoints; i += 2)
        {
          xCalibPoints[i / 2] = (CalibPoints[i] - storage.PSICalibCoeffs1) / storage.PSICalibCoeffs0;
          yCalibPoints[i / 2] = CalibPoints[i + 1];
        }

        //Compute linear regression coefficients
        simpLinReg(xCalibPoints, yCalibPoints, storage.PSICalibCoeffs0, storage.PSICalibCoeffs1, NbPoints / 2);

        //Store the new coefficients in eeprom
        saveParam("PSICalibCoeffs0",storage.PSICalibCoeffs0);
        saveParam("PSICalibCoeffs1",storage.PSICalibCoeffs1);          
        PublishSettings();
        Serial << F("Calibration completed. Coeffs are: ") << storage.PSICalibCoeffs0 << F(",") << storage.PSICalibCoeffs1 << _endl;
      }
    }
    //"Mode" command which sets regulation and filtration to manual or auto modes
    else if (command.containsKey(F("Mode")))
    {
      if ((int)command[F("Mode")] == 0)
      {
        storage.AutoMode = 0;

        //Stop PIDs
        SetPhPID(false);
        SetOrpPID(false);
      }
      else
      {
        storage.AutoMode = 1;
      }
      // saveParam("AutoMode",storage.AutoMode);
    }
    else if (command.containsKey(F("FiltPump"))) //"FiltPump" command which starts or stops the filtration pump
    {
      if ((int)command[F("FiltPump")] == 0)
      {
        EmergencyStopFiltPump = true;
        FiltrationPump.Stop();  //stop filtration pump

        //Stop PIDs
        SetPhPID(false);
        SetOrpPID(false);
      }
      else
      {
        EmergencyStopFiltPump = false;
        FiltrationPump.Start();   //start filtration pump
      }
    }
    else if (command.containsKey(F("RobotPump"))) //"RobotPump" command which starts or stops the Robot pump
    {
      if ((int)command[F("RobotPump")] == 0){
        RobotPump.Stop();    //stop robot pump
        Serial.println("Robot Stop commandé");
      } else {
        RobotPump.Start();   //start robot pump
        Serial.println("Robot Start commandé");
      }  
    }
    else if (command.containsKey(F("PhPump"))) //"PhPump" command which starts or stops the Acid pump
    {
      if ((int)command[F("PhPump")] == 0)
        PhPump.Stop();       //stop Acid pump
      else
        PhPump.Start();      //start Acid pump
    }
    else if (command.containsKey(F("ChlPump"))) //"ChlPump" command which starts or stops the Acid pump
    {
      if ((int)command[F("ChlPump")] == 0)
        ChlPump.Stop();      //stop Chl pump
      else
        ChlPump.Start();     //start Chl pump
    }
    else if (command.containsKey(F("PhPID"))) //"PhPID" command which starts or stops the Ph PID loop
    {
      if ((int)command[F("PhPID")] == 0)
      {
        //Stop PID
        SetPhPID(false);
      }
      else
      {
        //Initialize PIDs StartTime
        storage.PhPIDwindowStartTime = millis();
        storage.OrpPIDwindowStartTime = millis();

        //Start PID
        SetPhPID(true);
      }
    }
    else if (command.containsKey(F("OrpPID"))) //"OrpPID" command which starts or stops the Orp PID loop
    {
      if ((int)command[F("OrpPID")] == 0)
      {
        //Stop PID
        SetOrpPID(false);
      }
      else
      {
        //Start PID
        SetOrpPID(true);
      }
    }
    else if (command.containsKey(F("PhSetPoint"))) //"PhSetPoint" command which sets the setpoint for Ph
    {
      storage.Ph_SetPoint = (float)command[F("PhSetPoint")];
      saveParam("Ph_SetPoint",storage.Ph_SetPoint);
      PublishSettings();
    }
    else if (command.containsKey(F("OrpSetPoint"))) //"OrpSetPoint" command which sets the setpoint for ORP
    {
      storage.Orp_SetPoint = (float)command[F("OrpSetPoint")];
      saveParam("Orp_SetPoint",storage.Orp_SetPoint);
      PublishSettings();
    }
    else if (command.containsKey(F("WSetPoint"))) //"WSetPoint" command which sets the setpoint for Water temp (currently not in use)
    {
      storage.WaterTemp_SetPoint = (float)command[F("WSetPoint")];
      saveParam("WaterTempSet",storage.WaterTemp_SetPoint);
      PublishSettings();
    }
    //"pHTank" command which is called when the pH tank is changed or refilled
    //First parameter is volume of tank in Liters, second parameter is percentage Fill of the tank (typically 100% when new)
    else if (command.containsKey(F("pHTank")))
    {
      storage.pHTankVol = (float)command[F("pHTank")][0];
      PhPump.SetTankVolume(storage.pHTankVol);
      storage.AcidFill = (float)command[F("pHTank")][1];
      PhPump.ResetUpTime();
      saveParam("pHTankVol",storage.pHTankVol);
      saveParam("AcidFill",storage.AcidFill);               
      PublishSettings();
    }
    //"ChlTank" command which is called when the Chl tank is changed or refilled
    //First parameter is volume of tank in Liters, second parameter is percentage Fill of the tank (typically 100% when new)
    else if (command.containsKey(F("ChlTank")))
    {
      storage.ChlTankVol = (float)command[F("ChlTank")][0];
      ChlPump.SetTankVolume(storage.ChlTankVol);
      storage.ChlFill = (float)command[F("ChlTank")][1];
      ChlPump.ResetUpTime();
      saveParam("ChlTankVol",storage.ChlTankVol);
      saveParam("ChlFill",storage.ChlFill);
      PublishSettings();
    }
    else if (command.containsKey(F("WTempLow"))) //"WTempLow" command which sets the setpoint for Water temp low threshold
    {
      storage.WaterTempLowThreshold = (float)command[F("WTempLow")];
      saveParam("WaterTempLow",storage.WaterTempLowThreshold);
      PublishSettings();
    }
    else if (command.containsKey(F("PumpsMaxUp"))) //"PumpsMaxUp" command which sets the Max UpTime for pumps
    {
      storage.PhPumpUpTimeLimit = (unsigned int)command[F("PumpsMaxUp")];
      PhPump.SetMaxUpTime(storage.PhPumpUpTimeLimit * 1000);
      storage.ChlPumpUpTimeLimit = (unsigned int)command[F("PumpsMaxUp")];
      ChlPump.SetMaxUpTime(storage.ChlPumpUpTimeLimit * 1000);
      saveParam("PhPumpUTL",storage.PhPumpUpTimeLimit);
      saveParam("ChlPumpUTL",storage.ChlPumpUpTimeLimit);                    
      PublishSettings();
    }
    else if (command.containsKey(F("OrpPIDParams"))) //"OrpPIDParams" command which sets the Kp, Ki and Kd values for Orp PID loop
    {
      storage.Orp_Kp = (double)command[F("OrpPIDParams")][0];
      storage.Orp_Ki = (double)command[F("OrpPIDParams")][1];
      storage.Orp_Kd = (double)command[F("OrpPIDParams")][2];
      saveParam("Orp_Kp",storage.Orp_Kp);
      saveParam("Orp_Ki",storage.Orp_Ki);
      saveParam("Orp_Kd",storage.Orp_Kd);
      OrpPID.SetTunings(storage.Orp_Kp, storage.Orp_Ki, storage.Orp_Kd);
      PublishSettings();
    }
    else if (command.containsKey(F("PhPIDParams"))) //"PhPIDParams" command which sets the Kp, Ki and Kd values for Ph PID loop
    {
      storage.Ph_Kp = (double)command[F("PhPIDParams")][0];
      storage.Ph_Ki = (double)command[F("PhPIDParams")][1];
      storage.Ph_Kd = (double)command[F("PhPIDParams")][2];
      saveParam("Ph_Kp",storage.Ph_Kp);
      saveParam("Ph_Ki",storage.Ph_Ki);
      saveParam("Ph_Kd",storage.Ph_Kd);
      PhPID.SetTunings(storage.Ph_Kp, storage.Ph_Ki, storage.Ph_Kd);
      PublishSettings();
    }
    else if (command.containsKey(F("OrpPIDWSize"))) //"OrpPIDWSize" command which sets the window size of the Orp PID loop
    {
      storage.OrpPIDWindowSize = (unsigned long)command[F("OrpPIDWSize")];
      saveParam("OrpPIDWSize",storage.OrpPIDWindowSize);
      PublishSettings();
    }
    else if (command.containsKey(F("PhPIDWSize"))) //"PhPIDWSize" command which sets the window size of the Ph PID loop
    {
      storage.PhPIDWindowSize = (unsigned long)command[F("PhPIDWSize")];
      saveParam("PhPIDWSize",storage.PhPIDWindowSize);
      PublishSettings();
    }
    else if (command.containsKey(F("Date"))) //"Date" command which sets the Date of RTC module
    {
      /*   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
      rtc.adjust(DateTime((uint8_t)command[F("Date")][3], (uint8_t)command[F("Date")][2], (uint8_t)command[F("Date")][0], (uint8_t)command[F("Date")][4], (uint8_t)command[F("Date")][5], (uint8_t)command[F("Date")][6]));
      */

      setTime((uint8_t)command[F("Date")][4], (uint8_t)command[F("Date")][5], (uint8_t)command[F("Date")][6], (uint8_t)command[F("Date")][0], (uint8_t)command[F("Date")][2], (uint8_t)command[F("Date")][3]); //(Day of the month, Day of the week, Month, Year, Hour, Minute, Second)
    }
    else if (command.containsKey(F("FiltT0"))) //"FiltT0" command which sets the earliest hour when starting Filtration pump
    {
      storage.FiltrationStartMin = (unsigned int)command[F("FiltT0")];
      saveParam("FiltrStartMin",storage.FiltrationStartMin);
      PublishSettings();
    }
    else if (command.containsKey(F("FiltT1"))) //"FiltT1" command which sets the latest hour for running Filtration pump
    {
      storage.FiltrationStopMax = (unsigned int)command[F("FiltT1")];
      saveParam("FiltrStopMax",storage.FiltrationStopMax);
      PublishSettings();
    }
    else if (command.containsKey(F("PubPeriod"))) //"PubPeriod" command which sets the periodicity for publishing system info to MQTT broker
    {
      storage.PublishPeriod = (unsigned long)command[F("PubPeriod")] * 1000; //in secs
      setPublishPeriod(storage.PublishPeriod); //in msecs
      saveParam("PublishPeriod",storage.PublishPeriod);
      PublishSettings();
    }
    else if (command.containsKey(F("Clear"))) //"Clear" command which clears the UpTime and pressure errors of the Pumps
    {
      if (PSIError)
        PSIError = false;

      if (PhPump.UpTimeError)
        PhPump.ClearErrors();

      if (ChlPump.UpTimeError)
        ChlPump.ClearErrors();

      mqttErrorPublish("");

      //start filtration pump if within scheduled time slots
      if (!EmergencyStopFiltPump && storage.AutoMode && (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop))
        FiltrationPump.Start();
    }
    else if (command.containsKey(F("DelayPID"))) //"DelayPID" command which sets the delay from filtering start before PID loops start regulating
    {
      storage.DelayPIDs = (unsigned int)command[F("DelayPID")];
      saveParam("DelayPIDs",storage.DelayPIDs);
      PublishSettings();
    }
    else if (command.containsKey(F("PSIHigh"))) //"PSIHigh" command which sets the water high-pressure threshold
    {
      storage.PSI_HighThreshold = (float)command[F("PSIHigh")];
      saveParam("PSI_High",storage.PSI_HighThreshold);
      PublishSettings();
    }
    //"Relay" command which is called to actuate relays from the CONTROLLINO.
    //Parameter 1 is the relay number (R0 in this example), parameter 2 is the relay state (ON in this example).
    else if (command.containsKey(F("Relay")))
    {
      switch ((int)command[F("Relay")][0])
      {
        case 0:
          (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R0, LOW) : digitalWrite(RELAY_R0, HIGH);
          break;
        case 1:
          (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R1, LOW) : digitalWrite(RELAY_R1, HIGH);
          break;
        case 2:
          (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R2, LOW) : digitalWrite(RELAY_R2, HIGH);
          break;
      }
    }
    else if (command.containsKey(F("Reboot")))//"Reboot" command forces a reboot of the controller
    {
      vTaskDelay(10000 / portTICK_PERIOD_MS); // wait 10s then restart. Other tasks continue.
      esp_restart();
    }
    else if (command.containsKey(F("pHPumpFR")))//"PhPumpFR" set flow rate of Ph pump
    {
      storage.pHPumpFR = (float)command[F("pHPumpFR")];
      PhPump.SetFlowRate((float)command[F("pHPumpFR")]);
      saveParam("pHPumpFR",storage.pHPumpFR);
      PublishSettings();
    }
    else if (command.containsKey(F("ChlPumpFR")))//"ChlPumpFR" set flow rate of Chl pump
    {
      storage.ChlPumpFR = (float)command[F("ChlPumpFR")];
      ChlPump.SetFlowRate((float)command[F("ChlpumpFR")]);
      saveParam("ChlPumpFR",storage.ChlPumpFR);
      PublishSettings();
    }
    else if (command.containsKey(F("RstpHCal")))//"RstpHCal" reset the calibration coefficients of the pH probe
    {
      storage.pHCalibCoeffs0 = 4.3;
      storage.pHCalibCoeffs1 = -2.63;
      saveParam("pHCalibCoeffs0",storage.pHCalibCoeffs0);
      saveParam("pHCalibCoeffs1",storage.pHCalibCoeffs1);
      PublishSettings();
    }
    else if (command.containsKey(F("RstOrpCal")))//"RstOrpCal" reset the calibration coefficients of the Orp probe
    {
      storage.OrpCalibCoeffs0 = -1189;
      storage.OrpCalibCoeffs1 = 2564;
      saveParam("OrpCalibCoeffs0",storage.OrpCalibCoeffs0);
      saveParam("OrpCalibCoeffs1",storage.OrpCalibCoeffs1);
      PublishSettings();
    }
    else if (command.containsKey(F("RstPSICal")))//"RstPSICal" reset the calibration coefficients of the pressure sensor
    {
      storage.PSICalibCoeffs0 = 1.11;
      storage.PSICalibCoeffs1 = 0;
      saveParam("PSICalibCoeffs0",storage.PSICalibCoeffs0);
      saveParam("PSICalibCoeffs1",storage.PSICalibCoeffs1);
      PublishSettings();
    }

    //Publish/Update on the MQTT broker the status of our variables
    PublishDataCallback(NULL);
  }
}

//Linear regression coefficients calculation function
// pass x and y arrays (pointers), lrCoef pointer, and n.
//The lrCoef array is comprised of the slope=lrCoef[0] and intercept=lrCoef[1].  n is the length of the x and y arrays.
//http://jwbrooks.blogspot.com/2014/02/arduino-linear-regression-function.html
void simpLinReg(float * x, float * y, double & lrCoef0, double & lrCoef1, int n)
{
  // initialize variables
  float xbar = 0;
  float ybar = 0;
  float xybar = 0;
  float xsqbar = 0;

  // calculations required for linear regression
  for (int i = 0; i < n; i++)
  {
    xbar += x[i];
    ybar += y[i];
    xybar += x[i] * y[i];
    xsqbar += x[i] * x[i];
  }

  xbar /= n;
  ybar /= n;
  xybar /= n;
  xsqbar /= n;

  // simple linear regression algorithm
  lrCoef0 = (xybar - xbar * ybar) / (xsqbar - xbar * xbar);
  lrCoef1 = ybar - lrCoef0 * xbar;
}