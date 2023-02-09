/*

  Tom Button Functions
 
*/

#include <Arduino.h>
#include "ILI9486_SPI.h"
//#include "ILI9486_SPI.h"
#include "Tom_Button.h"

ILI9486_SPI* LocalTft = NULL;

const int MaxButtons = 50;

typedef struct {
  int      bxPos      = 0;
  int      byPos      = 0;
  int      bWidth     = 5;
  int      bHeight    = 5;
  char     bText[12]  = "bText ";
  int      bState     = -1;        // -1 = Pushbutton | 0 = UP | 1 = Down
  bool     AutoRepeat = false;
  bool     OnOff      = false;
  bool     Visible    = true;
  uint16_t bColor     = 0;         // Button Background
  uint16_t tColor     = 0;         // Text Color
  void (*onClicked)() = nullptr;
  void (*onDown)()    = nullptr;
  void (*onUp)()      = nullptr;
} t_Button;

int NumButtons = 0;
t_Button Button[MaxButtons];


void SetOnClick(int btn, callback_funct* aFunction) {
  //Button[btn].onClicked = &aFunction;
}


void SetButtonText(int btn, String Text) {
  Text.toCharArray(Button[btn].bText, 12);
}

void SetAutoRepeat(int btn, bool State) {
  Button[btn].AutoRepeat = State;
}

bool GetAutoRepeat(int btn) {
  return Button[btn].AutoRepeat;
}

void SetButtonState(int btn, int State) {
  Button[btn].bState = State;
  DrawButtonOnState(btn);
  //Serial.print("Button "); Serial.print(btn); Serial.print(" set to "); Serial.println(State);
}

int GetButtonState(int btn) {
  return Button[btn].bState;
}

void ToggleButtonState(int btn) {
  if (Button[btn].bState == 0) {
    Button[btn].bState = 1;
  } else {
    Button[btn].bState = 0;
  }
}

bool GetButtonOnOff(int btn) {
  return Button[btn].OnOff;  
}

void ClearButtons(void) {
  NumButtons = 0;  
}

void ButtonBegin(ILI9486_SPI* MyTft) {
  LocalTft = MyTft;
  //Serial.print("Size of LocalTft = "); Serial.println(sizeof(LocalTft));
  //Serial.print("Size of MyTft = "); Serial.println(sizeof(MyTft));
}

void SetButtonVisibility(int btn, bool Visibility) {
  Button[btn].Visible = Visibility;
}

String GetButtonText(int btn) {
  return Button[btn].bText;
}

void DrawDown(int btn) {
  int16_t  x1 = 0;
  int16_t  y1 = 0;
  uint16_t  w = 0;
  uint16_t  h = 0;

  if (Button[btn].Visible) {
    LocalTft->getTextBounds(Button[btn].bText, 0, 0, &x1, &y1, &w, &h);
    LocalTft->fillRect(Button[btn].bxPos, Button[btn].byPos, Button[btn].bWidth, Button[btn].bHeight, Button[btn].bColor);
    if (Button[btn].OnOff) {
      LocalTft->fillRect(Button[btn].bxPos+3, Button[btn].byPos+3, 5, 5, BLACK);
      LocalTft->fillRect(Button[btn].bxPos+4, Button[btn].byPos+4, 3, 3, GREEN);
    }  
    LocalTft->drawFastHLine(Button[btn].bxPos+1, Button[btn].byPos+1, Button[btn].bWidth, DARKGREY);
    LocalTft->drawFastVLine(Button[btn].bxPos+1, Button[btn].byPos+1, Button[btn].bHeight, DARKGREY);
    LocalTft->drawFastHLine(Button[btn].bxPos+1, Button[btn].byPos+Button[btn].bHeight, Button[btn].bWidth, WHITE);
    LocalTft->drawFastVLine(Button[btn].bxPos+Button[btn].bWidth, Button[btn].byPos+1, Button[btn].bHeight, WHITE);
    LocalTft->setCursor(Button[btn].bxPos+(Button[btn].bWidth/2)-(w/2)+3, Button[btn].byPos+(Button[btn].bHeight/2)-(h/2)+3);
    LocalTft->print(Button[btn].bText);
  } //else LocalTft->drawRect(Button[btn].bxPos, Button[btn].byPos, Button[btn].bWidth, Button[btn].bHeight, Button[btn].bColor); 
}

void DrawUp(int btn) {
  int16_t  x1 = 0;
  int16_t  y1 = 0;
  uint16_t  w = 0;
  uint16_t  h = 0;
  
  if (Button[btn].Visible) {
    LocalTft->fillRect(Button[btn].bxPos, Button[btn].byPos, Button[btn].bWidth, Button[btn].bHeight, Button[btn].bColor);
    if (Button[btn].OnOff) {
      LocalTft->fillRect(Button[btn].bxPos+3, Button[btn].byPos+3, 5, 5, BLACK);
      LocalTft->fillRect(Button[btn].bxPos+4, Button[btn].byPos+4, 3, 3, DARKGREY);
    }  
    
    LocalTft->getTextBounds(Button[btn].bText, 0, 0, &x1, &y1, &w, &h);
    //Serial.println(w);
    LocalTft->setCursor(Button[btn].bxPos+(Button[btn].bWidth/2)-(w/2)+2, Button[btn].byPos+(Button[btn].bHeight/2)-(h/2)+2);
    LocalTft->print(Button[btn].bText);
    LocalTft->drawFastHLine(Button[btn].bxPos+1, Button[btn].byPos+1, Button[btn].bWidth, WHITE);
    LocalTft->drawFastVLine(Button[btn].bxPos+1, Button[btn].byPos+1, Button[btn].bHeight, WHITE);
    LocalTft->drawFastHLine(Button[btn].bxPos+1, Button[btn].byPos+Button[btn].bHeight, Button[btn].bWidth, DARKGREY);
    LocalTft->drawFastVLine(Button[btn].bxPos+Button[btn].bWidth, Button[btn].byPos+1, Button[btn].bHeight, DARKGREY);
  } //else LocalTft->drawRect(Button[btn].bxPos, Button[btn].byPos, Button[btn].bWidth, Button[btn].bHeight, Button[btn].bColor); 
}

void DrawButtonOnState(int btn) {
  //Serial.print("Draw Button "); Serial.print(btn); Serial.print(" State = "); Serial.println(Button[btn].bState);
  if (Button[btn].bState == 0) {
    DrawUp(btn); 
  } else {
    DrawDown(btn); 
  }
}

void DrawButtons(int sel) {
  LocalTft->setTextSize(2);
  // Redraw Pushed Button depending on bState
  //Serial.print("Sel = "); Serial.println(sel);
  if (sel > -1) {
    //Serial.print("bState = "); Serial.println(Button[sel].bState);
    LocalTft->setTextColor(Button[sel].tColor);  
    if (Button[sel].bState == 1) DrawDown(sel); else DrawUp(sel);
  } else 
  {
    // Draw all Buttons
    for (int btn=0; btn <NumButtons; btn++) {
      if (Button[btn].bState == -1) {
        LocalTft->setTextColor(Button[btn].tColor);  
        DrawUp(btn);
      } else
      if (Button[btn].bState == 1) {
        LocalTft->setTextColor(Button[btn].tColor);  
        DrawDown(btn);
      } else 
      if (Button[btn].bState == 0) {
        LocalTft->setTextColor(Button[btn].tColor);  
        DrawUp(btn);
      }
    }
  }
}

bool ButtonAdd(int xPos, int yPos, int width, int height, int State, bool OnOff, bool Vis, String Text, uint16_t bColor, uint16_t tColor) {
  if (NumButtons < MaxButtons) {
    Button[NumButtons].bxPos      = xPos;
    Button[NumButtons].byPos      = yPos;
    Button[NumButtons].bWidth     = width;
    Button[NumButtons].bHeight    = height;
    Button[NumButtons].bState     = State;
    Button[NumButtons].OnOff      = OnOff;
    Button[NumButtons].Visible    = Vis;
    Button[NumButtons].AutoRepeat = false;
    Button[NumButtons].bColor     = bColor;
    Button[NumButtons].tColor     = tColor;
    Text.toCharArray(Button[NumButtons].bText, 12);
    NumButtons += 1;
    return true;
  } else {
    return false;
  }
}

int GetButtonByPos(int x, int y) {  
  int result = -1;
  for (int btn=0; btn <NumButtons; btn++) {
    if ((x >= Button[btn].bxPos) && (x <= (Button[btn].bxPos+Button[btn].bWidth))) {
      if ((y >= Button[btn].byPos) && (y <= (Button[btn].byPos+Button[btn].bHeight))) {
        result = btn;
        if (Button[btn].onClicked) Button[btn].onClicked();
        break;
      }
    }
  }
  return result;
}
