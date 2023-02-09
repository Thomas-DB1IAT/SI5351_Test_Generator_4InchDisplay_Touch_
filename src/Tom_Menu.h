/*
    Functionen zur Verwendung von MENUs 
    auf einem Touchscreen
*/

#include <Arduino.h>
#include "ILI9486_SPI.h"

typedef struct {
  char     CallSign[12] = "DB1IAT";
  char     OpName[12]   = "THOMAS";
  char     Locator[12]  = "JN48DX";
  int      CwSpeed      = 20;
} t_Settings;



void Tom_MenuBegin(ILI9486_SPI* MyTft);
