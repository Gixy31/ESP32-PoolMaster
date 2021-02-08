#include "EasyNextionLibrary.h"

int CurrentPage = 0;
char HourBuffer[8];
uint8_t debounceCount = 2;
uint8_t debounceM = 0;
uint8_t debounceF = 0;
uint8_t debounceH = 0;
uint8_t debounceR0 = 0;
uint8_t debounceR1 = 0;
uint8_t debounceR2 = 0;

// Structure holding the measurement values to display on the Nextion display
// Used to refresh only modified values
struct TFTStruct
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
EasyNex myNex(Serial2);