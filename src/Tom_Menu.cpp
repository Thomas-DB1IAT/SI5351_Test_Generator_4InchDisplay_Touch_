/*

  Tom Menu Functions
 
*/

#include <Arduino.h>
#include "ILI9486_SPI.h"
//#include "ILI9486_SPI.h"
#include "Tom_Menu.h"

ILI9486_SPI* MenuTft = NULL;

void Tom_MenuBegin(ILI9486_SPI* MyTft) {
  MenuTft = MyTft;
}
