/*
    Functionen zur Verwendung von Buttons 
    auf einem Touchscreen
*/

#include <Arduino.h>
#include "ILI9486_SPI.h"

#define BLACK     0x0000
#define DARKGREY  0x7BEF
#define LIGHTGREY 0xC618
#define BLUE      0x001F
#define RED       0xF800
#define GREEN     0x07E0
#define CYAN      0x07FF
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0
#define WHITE     0xFFFF

typedef void (*callback_funct)(void); // type for conciseness

void ButtonBegin(ILI9486_SPI* MyTft);
bool ButtonAdd(int xPos, int yPos, int width, int height,  int State, bool OnOff, String Text, uint16_t bColor, uint16_t tColor);
void ClearButtons(void);
void DrawButtonOnState(int btn);
void DrawButtons(int sel);
void DrawDown(int btn);
void DrawUp(int btn);

void SetOnClick(int btn, callback_funct aFunction);

int  GetButtonByPos(int x, int y);

void SetAutoRepeat(int btn, bool State);
bool GetAutoRepeat(int btn);

void SetButtonText(int btn, String Text);
String GetButtonText(int btn); 

void SetButtonState(int btn, int State);
int  GetButtonState(int btn);

// If the Button has the OnOff state it can be down even not touched like a Switch
bool GetButtonOnOff(int btn);
// If the ButtonVisibility is set to false the Button exists but is not drawed
void SetButtonVisibility(int btn, bool Visibility);
void ToggleButtonState(int btn);
