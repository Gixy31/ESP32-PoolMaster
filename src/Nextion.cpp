/*
  NEXTION TFT related code, based on EasyNextion library by Seithan / Athanasios Seitanis (https://github.com/Seithan/EasyNextionLibrary)
  The trigger(s) functions at the end are called by the Nextion library on event (buttons, page change).

  (c) Loic74 <loic74650@gmail.com> 2018-2020
*/

#include <Arduino.h>
#include "config.h"
#include "PoolMaster.h"
#include "Nextion.h"

String temp;

void InitTFT(void);
void ResetTFT(void);
void UpdateTFT(void);
void UpdateWiFi(bool);


void InitTFT()
{
  myNex.begin(9600);
}

//reset TFT at start of controller
void ResetTFT()
{
  myNex.writeStr(F("rest"));
}

void UpdateWiFi(bool wifi){
  if(wifi){
    temp = "WiFi: " + WiFi.SSID();
    myNex.writeStr("page0.vaSSID.txt",temp);
    temp = "IP: " + WiFi.localIP().toString();
    myNex.writeStr("page0.vaIP.txt",temp);
  } else
  {
    myNex.writeStr("page0.vaSSID.txt","not connected");
    myNex.writeStr("page0.vaIP.txt","");
  } 
}

//function to update TFT display
//it updates the TFTStruct variables, the global variables of the TFT + the widgets of the active page
//call this function at least every second to ensure fluid display
void UpdateTFT()
{
  myNex.NextionListen();

  sprintf(HourBuffer, "%02d:%02d:%02d", hour(), minute(), second());
  myNex.writeStr("page0.vaTime.txt", HourBuffer);

  if (Firmw != TFTStruc.FW)
  {
    TFTStruc.FW = F("MC fw: v ");
    TFTStruc.FW += Firmw;
    myNex.writeStr(F("page0.vaMCFW.txt"), TFTStruc.FW);
  }

  if (MQTTConnection != TFTStruc.NetW)
  {
    TFTStruc.NetW = MQTTConnection;
    myNex.writeNum(F("page1.vabNetW.val"), TFTStruc.NetW);

    if (CurrentPage == 0)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p0NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p0NetW.pic=6"));
      }
    }
    else if (CurrentPage == 1)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p1NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p1NetW.pic=6"));
      }
    }
    else if (CurrentPage == 2)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p2NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p2NetW.pic=6"));
      }
    }
    else if (CurrentPage == 3)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
  }

  if (storage.PhValue != TFTStruc.pH)
  {
    TFTStruc.pH = storage.PhValue;
    myNex.writeStr(F("page0.vapH.txt"), String(TFTStruc.pH, 2));
    if (CurrentPage == 0)  myNex.writeStr(F("pH.txt"), String(TFTStruc.pH, 2));
  }
  if (storage.OrpValue != TFTStruc.Orp)
  {
    TFTStruc.Orp = storage.OrpValue;
    myNex.writeStr(F("page0.vaOrp.txt"), String(TFTStruc.Orp, 0));
    if (CurrentPage == 0)  myNex.writeStr(F("Orp.txt"), String(TFTStruc.Orp, 0));
  }
  if (storage.Ph_SetPoint != TFTStruc.pHSP)
  {
    TFTStruc.pHSP = storage.Ph_SetPoint;
    temp = "(" + String(TFTStruc.pHSP, 1) + ")";
    myNex.writeStr(F("page0.vapHSP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("pHSP.txt"), temp);
  }
  if (storage.Orp_SetPoint != TFTStruc.OrpSP)
  {
    TFTStruc.OrpSP = storage.Orp_SetPoint;
    temp = "(" + String((int)TFTStruc.OrpSP) + ")";
    myNex.writeStr(F("page0.vaOrpSP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("OrpSP.txt"), temp);
  }
  if (storage.TempValue != TFTStruc.WT)
  {
    TFTStruc.WT = storage.TempValue;
    temp = String(TFTStruc.WT, 1) + (char)176 + F("C");
    myNex.writeStr(F("page0.vaWT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("W.txt"), temp);
  }
  if (storage.TempExternal != TFTStruc.AT)
  {
    TFTStruc.AT = storage.TempExternal;
    temp = String(TFTStruc.AT, 1) + (char)176 + F("C");
    myNex.writeStr(F("page0.vaAT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("A.txt"), temp);
  }
  if (storage.PSIValue != TFTStruc.PSI)
  {
    TFTStruc.PSI = storage.PSIValue;
    temp = String(TFTStruc.PSI, 2) + F("b");
    myNex.writeStr(F("page0.vaPSI.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("P.txt"), temp);
  }

  if ((storage.FiltrationStop != TFTStruc.FSto) || (storage.FiltrationStart != TFTStruc.FSta))
  {
    TFTStruc.FSto = storage.FiltrationStop;
    TFTStruc.FSta = storage.FiltrationStart;
    temp = String(TFTStruc.FSta) + F("/") + String(TFTStruc.FSto) + F("h");
    myNex.writeStr(F("page0.vaStaSto.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("page0.p0StaSto.txt"), temp);
    else if (CurrentPage == 1)  myNex.writeStr(F("page1.p1StaSto.txt"), temp);
    else if (CurrentPage == 2)  myNex.writeStr(F("page2.p2StaSto.txt"), temp);
    else if (CurrentPage == 3)  myNex.writeStr(F("page3.p3StaSto.txt"), temp);
  }

  if ((ChlPump.UpTime != TFTStruc.OrpPpRT) || ((int)(storage.ChlFill - ChlPump.GetTankUsage()) != TFTStruc.OrpTkFill))
  {
    TFTStruc.OrpPpRT = ChlPump.UpTime;
    TFTStruc.OrpTkFill = (int)round((storage.ChlFill - ChlPump.GetTankUsage()));

    temp = String(TFTStruc.OrpTkFill) + (char)37 + F(" / ") + String(float(TFTStruc.OrpPpRT / 1000 / 60), 1) + F("min");
    myNex.writeStr(F("page0.vaOrpTk.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("OrpTk.txt"), temp);
  }

  if ((PhPump.UpTime != TFTStruc.pHPpRT) || ((int)(storage.AcidFill - PhPump.GetTankUsage()) != TFTStruc.pHTkFill))
  {
    TFTStruc.pHPpRT = PhPump.UpTime;
    TFTStruc.pHTkFill = (int)round((storage.AcidFill - PhPump.GetTankUsage()));

    temp = String(TFTStruc.pHTkFill) + (char)37 + F(" / ") + String(float(TFTStruc.pHPpRT / 1000 / 60), 1) + F("min");
    myNex.writeStr(F("page0.vapHTk.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("pHTk.txt"), temp);
  }

  if (storage.AutoMode != TFTStruc.Mode)
  {
    if ((debounceM == 0) || (debounceM > debounceCount))
    {
      debounceM = 0;
      TFTStruc.Mode = storage.AutoMode;
      temp = TFTStruc.Mode ? F("AUTO") : F("MANU");
      myNex.writeNum(F("page1.vabMode.val"), storage.AutoMode);
      if (CurrentPage == 0)
      {
        myNex.writeStr(F("p0Mode.txt"), temp);
      }
      else if (CurrentPage == 1)
      {
        myNex.writeStr(F("p1Mode.txt"), temp);
        myNex.writeStr(F("t1Mode.txt"), temp);
        if (storage.AutoMode == 1)
          myNex.writeStr(F("bMode.pic=8"));
        else
          myNex.writeStr(F("bMode.pic=7"));
      }
      else if (CurrentPage == 2)
      {
        myNex.writeStr(F("p2Mode.txt"), temp);
      }
      else if (CurrentPage == 3)
      {
        myNex.writeStr(F("p3Mode.txt"), temp);
      }
    }
    else
      debounceM++;
  }

  if (FiltrationPump.IsRunning() != TFTStruc.Filt)
  {
    if ((debounceF == 0) || (debounceF > debounceCount))
    {
      debounceF = 0;
      TFTStruc.Filt = FiltrationPump.IsRunning();
      myNex.writeNum(F("page1.vabFilt.val"), TFTStruc.Filt);
      if (CurrentPage == 1)
      {
        if (TFTStruc.Filt == 1)
          myNex.writeStr(F("bFilt.pic=8"));
        else
          myNex.writeStr(F("bFilt.pic=7"));
      }
    }
    else
      debounceF++;
  }

  if (RobotPump.IsRunning() != TFTStruc.Robot)
  {
    if ((debounceH == 0) || (debounceH > debounceCount))
    {
      debounceH = 0;
      TFTStruc.Robot = RobotPump.IsRunning();
      myNex.writeNum(F("page1.vabRobot.val"), TFTStruc.Robot);
      if (CurrentPage == 1)
      {
        if (TFTStruc.Robot == 1)
          myNex.writeStr(F("bRobot.pic=8"));
        else
          myNex.writeStr(F("bRobot.pic=7"));
      }
    }
    else
      debounceH++;
  }

  if (digitalRead(RELAY_R0) != TFTStruc.R0)
  {
    if ((debounceR0 == 0) || (debounceR0 > debounceCount))
    {
      debounceR0 = 0;
      TFTStruc.R0 = digitalRead(RELAY_R0);
      myNex.writeNum(F("page1.vabR0.val"), !TFTStruc.R0);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R0 == 0)
          myNex.writeStr(F("bR0.pic=8"));
        else
          myNex.writeStr(F("bR0.pic=7"));
      }
    }
    else
      debounceR0++;
  }

  if (digitalRead(RELAY_R1) != TFTStruc.R1)
  {
    if ((debounceR1 == 0) || (debounceR1 > debounceCount))
    {
      debounceR1 = 0;
      TFTStruc.R1 = digitalRead(RELAY_R1);
      myNex.writeNum(F("page1.vabR1.val"), !TFTStruc.R1);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R1 == 0)
          myNex.writeStr(F("bR1.pic=8"));
        else
          myNex.writeStr(F("bR1.pic=7"));
      }
    }
    else
      debounceR1++;
  }

  if (digitalRead(RELAY_R2) != TFTStruc.R2)
  {
    if ((debounceR2 == 0) || (debounceR2 > debounceCount))
    {
      debounceR2 = 0;
      TFTStruc.R2 = digitalRead(RELAY_R2);

      myNex.writeNum(F("page1.vabR2.val"), !TFTStruc.R2);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R2 == 0)
          myNex.writeStr(F("bR2.pic=8"));
        else
          myNex.writeStr(F("bR2.pic=7"));
      }
    }
    else
      debounceR2++;
  }

  if (ChlPump.TankLevel() != TFTStruc.ChlTLErr)
  {
    TFTStruc.ChlTLErr = ChlPump.TankLevel();
    if (!TFTStruc.ChlTLErr)
    {
      myNex.writeStr(F("page0.vaChlLevel.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaChlLevel.val=0"));
  }

  if (PhPump.TankLevel() != TFTStruc.pHTLErr)
  {
    TFTStruc.pHTLErr = PhPump.TankLevel();
    if (!TFTStruc.pHTLErr)
    {
      myNex.writeStr(F("page0.vaAcidLevel.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaAcidLevel.val=0"));
  }

  if (PSIError != TFTStruc.PSIErr)
  {
    TFTStruc.PSIErr = PSIError;
    if (TFTStruc.PSIErr)
    {
      myNex.writeStr(F("page0.vaPSIErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaPSIErr.val=0"));
  }

  if (ChlPump.UpTimeError != TFTStruc.ChlUTErr)
  {
    TFTStruc.ChlUTErr = ChlPump.UpTimeError;
    if (TFTStruc.ChlUTErr)
    {
      myNex.writeStr(F("page0.vaChlUTErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaChlUTErr.val=0"));
  }

  if (PhPump.UpTimeError != TFTStruc.pHUTErr)
  {
    TFTStruc.pHUTErr = PhPump.UpTimeError;
    if (TFTStruc.pHUTErr)
    {
      myNex.writeStr(F("page0.vapHUTErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vapHUTErr.val=0"));
  }

  //update time at top of displayed page
  switch (CurrentPage)
  {
    case 0: {
        myNex.writeStr(F("p0Time.txt"), HourBuffer);
        break;
      }
    case 1: {
        myNex.writeStr(F("p1Time.txt"), HourBuffer);
        break;
      }
    case 2: {
        myNex.writeStr(F("p2Time.txt"), HourBuffer);
        break;
      }
    case 3: {
        myNex.writeStr(F("p3Time.txt"), HourBuffer);
        break;
      }
  }
}

//Page 0 has finished loading
//printh 23 02 54 01
void trigger1()
{
  CurrentPage = 0;
  DEBUG_PRINT(F("Nextion p0"));
}

//Page 1 has finished loading
//printh 23 02 54 02
void trigger2()
{
  CurrentPage = 1;
  DEBUG_PRINT(F("Nextion p1"));
}

//Page 2 has finished loading
//printh 23 02 54 03
void trigger3()
{
  CurrentPage = 2;
  DEBUG_PRINT(F("Nextion p2"));
}

//Page 3 has finished loading
////printh 23 02 54 04
void trigger4()
{
  CurrentPage = 3;
  DEBUG_PRINT(F("Nextion p3"));
}

//MODE button was toggled
//printh 23 02 54 05
void trigger5()
{
  TFTStruc.Mode = (boolean)myNex.readNumber(F("vabMode.val"));
  debounceM = 1;
  DEBUG_PRINT(F("MODE button"));
  if (TFTStruc.Mode)
  {
    String Cmd = F("{\"Mode\":1}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Mode\":0}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//FILT button was toggled
//printh 23 02 54 06
void trigger6()
{
  TFTStruc.Filt = (boolean)myNex.readNumber(F("vabFilt.val"));
  debounceF = 1;
  DEBUG_PRINT(F("FILT button"));
  if (TFTStruc.Filt)
  {
    String Cmd = F("{\"FiltPump\":1}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"FiltPump\":0}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Robot button was toggled
//printh 23 02 54 07
void trigger7()
{
  TFTStruc.Robot = (boolean)myNex.readNumber(F("vabRobot.val"));
  debounceH = 1;
  DEBUG_PRINT(F("Robot button"));
  if (TFTStruc.Robot)
  {
    String Cmd = F("{\"RobotPump\":1}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"RobotPump\":0}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Relay 0 button was toggled
//printh 23 02 54 08
void trigger8()
{
  TFTStruc.R0 = (boolean)myNex.readNumber(F("vabR0.val"));
  debounceR0 = 1;
  DEBUG_PRINT(F("Relay 0 button"));
  if (TFTStruc.R0)
  {
    String Cmd = F("{\"Relay\":[0,1]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Relay\":[0,0]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Relay 1 button was toggled
//printh 23 02 54 09
void trigger9()
{
  TFTStruc.R1 = (boolean)myNex.readNumber(F("vabR1.val"));
  debounceR1 = 1;
  DEBUG_PRINT(F("Relay 1 button"));
  if (TFTStruc.R1)
  {
    String Cmd = F("{\"Relay\":[1,1]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Relay\":[1,0]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Relay 2 button was toggled
//printh 23 02 54 0A
void trigger10()
{
  TFTStruc.R2 = (boolean)myNex.readNumber(F("vabR2.val"));
  debounceR2 = 1;
  DEBUG_PRINT(F("Relay 2 button"));
  if (TFTStruc.R2)
  {
    String Cmd = F("{\"Relay\":[2,1]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Relay\":[2,0]}");
    queueIn.enqueue(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Probe calibration completed or new pH, Orp or Water Temp setpoints or New tank
//printh 23 02 54 0B
void trigger11()
{
  DEBUG_PRINT("Calibration complete or new pH, Orp or Water Temp setpoints or new tank event");
  String Cmd = myNex.readStr(F("pageCalibs.vaCommand.txt"));
  queueIn.enqueue(Cmd);
  DEBUG_PRINT(Cmd);
}

//Clear Errors button pressed
//printh 23 02 54 0C
void trigger12()
{
  DEBUG_PRINT(F("Clear errors event"));
  String Cmd = F("{\"Clear\":1}");
  queueIn.enqueue(Cmd);
}
