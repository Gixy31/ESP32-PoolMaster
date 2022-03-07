/*
  NEXTION TFT related code, based on EasyNextion library by Seithan / Athanasios Seitanis (https://github.com/Seithan/EasyNextionLibrary)
  The trigger(s) functions at the end are called by the Nextion library on event (buttons, page change).

  (c) Loic74 <loic74650@gmail.com> 2018-2020

  Modified to implement display sleep mode.
*/

#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"
#include "EasyNextionLibrary.h"

static volatile int CurrentPage = 0;
static volatile bool TFT_ON = true;           // display status
static volatile bool refresh = false;         // flag to force display refresh

static String temp;
static unsigned long LastAction = 0; // Last action time done on TFT. Go to sleep after TFT_SLEEP
static char HourBuffer[9];

static uint8_t debounceCount = 2;
static uint8_t debounceM     = 0;
static uint8_t debounceF     = 0;
static uint8_t debounceH     = 0;
static uint8_t debounceR0    = 0;
static uint8_t debounceR1    = 0;
static uint8_t debounceR2    = 0;

// Structure holding the measurement values to display on the Nextion display
// Used to refresh only modified values
static struct TFTStruct
{
  float pH, Orp, pHSP, OrpSP, WT, WTSP, AT, PSI;
  uint8_t FSta, FSto, pHTkFill, OrpTkFill, PIDpH, PIDChl;
  boolean Mode, NetW, Filt, Robot, R0, R1, R2, pHUTErr, ChlUTErr, PSIErr, pHTLErr, ChlTLErr;
  unsigned long pHPpRT, OrpPpRT;
  String FW;
} TFTStruc =
{ //default values to force update on next refresh
  -1., -1., -1., -1., -1., -1., -1., -1.,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  99, 99,
  ""
};

//Nextion TFT object. Choose which ever Serial port
//you wish to connect to (not "Serial" which is used for debug), here Serial2 UART
static EasyNex myNex(Serial2);

// Functions prototypes
void InitTFT(void);
void ResetTFT(void);
void UpdateTFT(void);
void UpdateWiFi(bool);


void InitTFT()
{
  myNex.begin(9600);
}

//reset TFT at start of controller - Change transmission rate to 115200 bauds on both side (Nextion then ESP)
//could have been not in HMI file, but it is good to know that after reset the Nextion goes back to 9600 bauds
void ResetTFT()
{
  myNex.writeStr(F("rest"));
  delay(1000);
  myNex.writeStr("baud=115200");
  delay(100);
  myNex.begin(115200);
  LastAction = millis();
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

  if (MQTTConnection != TFTStruc.NetW || !refresh)
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

  if (storage.PhValue != TFTStruc.pH || !refresh)
  {
    TFTStruc.pH = storage.PhValue;
    myNex.writeStr(F("page0.vapH.txt"), String(TFTStruc.pH, 2));
    if (CurrentPage == 0)  myNex.writeStr(F("pH.txt"), String(TFTStruc.pH, 2));
  }
  if (storage.OrpValue != TFTStruc.Orp || !refresh)
  {
    TFTStruc.Orp = storage.OrpValue;
    myNex.writeStr(F("page0.vaOrp.txt"), String(TFTStruc.Orp, 0));
    if (CurrentPage == 0)  myNex.writeStr(F("Orp.txt"), String(TFTStruc.Orp, 0));
  }
  if (storage.Ph_SetPoint != TFTStruc.pHSP || !refresh)
  {
    TFTStruc.pHSP = storage.Ph_SetPoint;
    temp = "(" + String(TFTStruc.pHSP, 1) + ")";
    myNex.writeStr(F("page0.vapHSP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("pHSP.txt"), temp);
  }
  if (storage.Orp_SetPoint != TFTStruc.OrpSP || !refresh)
  {
    TFTStruc.OrpSP = storage.Orp_SetPoint;
    temp = "(" + String((int)TFTStruc.OrpSP) + ")";
    myNex.writeStr(F("page0.vaOrpSP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("OrpSP.txt"), temp);
  }
  if (PhPID.GetMode() != TFTStruc.PIDpH || !refresh)
  {
    TFTStruc.PIDpH = PhPID.GetMode();
    temp = (TFTStruc.PIDpH == 1) ? F("PID") : F("---");
    myNex.writeStr(F("page0.vapHP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("pHP.txt"), temp);
  }
  if (OrpPID.GetMode() != TFTStruc.PIDChl || !refresh)
  {
    TFTStruc.PIDChl = OrpPID.GetMode();
    temp = (TFTStruc.PIDChl == 1) ? F("PID") : F("---");
    myNex.writeStr(F("page0.vaChlP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("ChlP.txt"), temp);
  }
  if (storage.TempValue != TFTStruc.WT || !refresh)
  {
    TFTStruc.WT = storage.TempValue;
    temp = String(TFTStruc.WT, 1) + (char)176 + F("C");
    myNex.writeStr(F("page0.vaWT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("W.txt"), temp);
  }
  if (storage.TempExternal != TFTStruc.AT || !refresh)
  {
    TFTStruc.AT = storage.TempExternal;
    temp = String(TFTStruc.AT, 1) + (char)176 + F("C");
    myNex.writeStr(F("page0.vaAT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("A.txt"), temp);
  }
  if (storage.PSIValue != TFTStruc.PSI || !refresh)
  {
    TFTStruc.PSI = storage.PSIValue;
    temp = String(TFTStruc.PSI, 2) + F("b");
    myNex.writeStr(F("page0.vaPSI.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("P.txt"), temp);
  }

  if ((storage.FiltrationStop != TFTStruc.FSto) || (storage.FiltrationStart != TFTStruc.FSta) || !refresh)
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

  if ((ChlPump.UpTime != TFTStruc.OrpPpRT) || ((int)ChlPump.GetTankFill() != TFTStruc.OrpTkFill) || !refresh)
  {
    TFTStruc.OrpPpRT = ChlPump.UpTime;
    TFTStruc.OrpTkFill = (int)round(ChlPump.GetTankFill());

    temp = String(TFTStruc.OrpTkFill) + (char)37 + F(" / ") + String(float(TFTStruc.OrpPpRT)/1000./60., 1) + F("min");
    myNex.writeStr(F("page0.vaOrpTk.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("OrpTk.txt"), temp);
  }

  if ((PhPump.UpTime != TFTStruc.pHPpRT) || ((int)PhPump.GetTankFill() != TFTStruc.pHTkFill) || !refresh)
  {
    TFTStruc.pHPpRT = PhPump.UpTime;
    TFTStruc.pHTkFill = (int)round(PhPump.GetTankFill());

    temp = String(TFTStruc.pHTkFill) + (char)37 + F(" / ") + String(float(TFTStruc.pHPpRT)/1000./60., 1) + F("min");
    myNex.writeStr(F("page0.vapHTk.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("pHTk.txt"), temp);
  }

  if (storage.AutoMode != TFTStruc.Mode || !refresh)
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

  if (FiltrationPump.IsRunning() != TFTStruc.Filt || !refresh)
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

  if (RobotPump.IsRunning() != TFTStruc.Robot || !refresh)
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

  if (digitalRead(RELAY_R0) != TFTStruc.R0 || !refresh)
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

  if (digitalRead(RELAY_R1) != TFTStruc.R1 || !refresh)
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

  if (storage.WinterMode != TFTStruc.R2 || !refresh)
  {
    if ((debounceR2 == 0) || (debounceR2 > debounceCount))
    {
      debounceR2 = 0;
      TFTStruc.R2 = storage.WinterMode;
      myNex.writeNum(F("page1.vabR2.val"), TFTStruc.R2);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R2 == 1)
          myNex.writeStr(F("bR2.pic=8"));
        else
          myNex.writeStr(F("bR2.pic=7"));
      }
    }
    else
      debounceR2++;
  }

  if (ChlPump.TankLevel() != TFTStruc.ChlTLErr || !refresh)
  {
    TFTStruc.ChlTLErr = ChlPump.TankLevel();
    if (!TFTStruc.ChlTLErr)
    {
      myNex.writeStr(F("page0.vaChlLevel.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaChlLevel.val=0"));
  }

  if (PhPump.TankLevel() != TFTStruc.pHTLErr || !refresh)
  {
    TFTStruc.pHTLErr = PhPump.TankLevel();
    if (!TFTStruc.pHTLErr)
    {
      myNex.writeStr(F("page0.vaAcidLevel.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaAcidLevel.val=0"));
  }

  if (PSIError != TFTStruc.PSIErr || !refresh)
  {
    TFTStruc.PSIErr = PSIError;
    if (TFTStruc.PSIErr)
    {
      myNex.writeStr(F("page0.vaPSIErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaPSIErr.val=0"));
  }

  if (ChlPump.UpTimeError != TFTStruc.ChlUTErr || !refresh)
  {
    TFTStruc.ChlUTErr = ChlPump.UpTimeError;
    if (TFTStruc.ChlUTErr)
    {
      myNex.writeStr(F("page0.vaChlUTErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaChlUTErr.val=0"));
  }

  if (PhPump.UpTimeError != TFTStruc.pHUTErr || !refresh)
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
  //put TFT in sleep mode with wake up on touch and force page 0 load to trigger an event
  if((unsigned long)(millis() - LastAction) >= TFT_SLEEP && TFT_ON)
  {
    myNex.writeStr("thup=1");
    myNex.writeStr("wup=0");
    myNex.writeStr("sleep=1");
    TFT_ON = false;
  }
  refresh = true;
}

//Page 0 has finished loading
//printh 23 02 54 01
void trigger1()
{
  CurrentPage = 0;
  if(!TFT_ON)
  {
    UpdateWiFi(WiFi.status() == WL_CONNECTED);
    TFT_ON = true;  
    refresh = false;
  }
  LastAction = millis();
}

//Page 1 has finished loading
//printh 23 02 54 02
void trigger2()
{
  CurrentPage = 1;
  LastAction = millis();
}

//Page 2 has finished loading
//printh 23 02 54 03
void trigger3()
{
  CurrentPage = 2;
  LastAction = millis();  
}

//Page 3 has finished loading
//printh 23 02 54 04
void trigger4()
{
  CurrentPage = 3;
  LastAction = millis();
}

//MODE button was toggled
//printh 23 02 54 05
void trigger5()
{
  TFTStruc.Mode = (boolean)myNex.readNumber(F("vabMode.val"));
  debounceM = 1;
  Debug.print(DBG_VERBOSE,"MODE button");
  char Cmd[100] = "{\"Mode\":0}";
  if(TFTStruc.Mode) Cmd[8] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Mode cmd: %s",Cmd);
  LastAction = millis();
}

//FILT button was toggled
//printh 23 02 54 06
void trigger6()
{
  TFTStruc.Filt = (boolean)myNex.readNumber(F("vabFilt.val"));
  debounceF = 1;
  Debug.print(DBG_VERBOSE,"FILT button");
  char Cmd[100] = "{\"FiltPump\":0}";
  if(TFTStruc.Filt) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Filt command: %s",Cmd);
  LastAction = millis();
}

//Robot button was toggled
//printh 23 02 54 07
void trigger7()
{
  TFTStruc.Robot = (boolean)myNex.readNumber(F("vabRobot.val"));
  debounceH = 1;
  Debug.print(DBG_VERBOSE,"Robot button");
  char Cmd[100] = "{\"RobotPump\":0}";
  if(TFTStruc.Robot) Cmd[13] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Robot command: %s",Cmd);
  LastAction = millis();
}

//Relay 0 button was toggled
//printh 23 02 54 08
void trigger8()
{
  TFTStruc.R0 = (boolean)myNex.readNumber(F("vabR0.val"));
  debounceR0 = 1;
  Debug.print(DBG_VERBOSE,"Relay 0 button");
  char Cmd[100] = "{\"Relay\":[0,0]}";
  if(TFTStruc.R0) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion R0 command: %s",Cmd);
  LastAction = millis();
}

//Relay 1 button was toggled
//printh 23 02 54 09
void trigger9()
{
  TFTStruc.R1 = (boolean)myNex.readNumber(F("vabR1.val"));
  debounceR1 = 1;
  Debug.print(DBG_VERBOSE,"Relay 1 button");
  char Cmd[100] = "{\"Relay\":[1,0]}";
  if(TFTStruc.R1) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion R1 command: %s",Cmd);
  LastAction = millis();
}

//Winter button was toggled
//printh 23 02 54 0A
void trigger10()
{
  TFTStruc.R2 = (boolean)myNex.readNumber(F("vabR2.val"));
  debounceR2 = 1;
  Debug.print(DBG_VERBOSE,"Winter button");
  char Cmd[100] = "{\"Winter\":0}";
  if(TFTStruc.R2) Cmd[10] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Winter command: %s",Cmd);
  LastAction = millis();
}

//Probe calibration completed or new pH, Orp or Water Temp setpoints or New tank
//printh 23 02 54 0B
void trigger11()
{
  Debug.print(DBG_VERBOSE,"Calibration complete or new pH, Orp or Water Temp setpoints or new tank event");
  char Cmd[100] = "";
  strcpy(Cmd,myNex.readStr(F("pageCalibs.vaCommand.txt")).c_str());
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion cal page command: %s",Cmd);
  LastAction = millis();
}

//Clear Errors button pressed
//printh 23 02 54 0C
void trigger12()
{
  Debug.print(DBG_VERBOSE,"Clear errors event");
  char Cmd[100] = "{\"Clear\":1}";
  xQueueSendToBack(queueIn,&Cmd,0);
  LastAction = millis();
}

//pH PID button pressed
//printh 23 02 54 0D
void trigger13()
{
  Debug.print(DBG_VERBOSE,"pH PID event");
  char Cmd[100] = "{\"PhPID\":1}";
  if(TFTStruc.PIDpH == 1) Cmd[9] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  LastAction = millis();
}

//Orp PID button pressed
//printh 23 02 54 0E
void trigger14()
{
  Debug.print(DBG_VERBOSE,"Orp PID event");
  char Cmd[100] = "{\"OrpPID\":1}";
  if(TFTStruc.PIDChl == 1) Cmd[10] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  LastAction = millis();
}