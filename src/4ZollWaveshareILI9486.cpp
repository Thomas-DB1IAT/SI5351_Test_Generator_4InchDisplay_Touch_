/*

  Frequency Generator ESP32 and Si5351 PLL with GPS

  ILI9341 incl. Touch
  GPS
  Si5351 PLL
  Rotary Encoder

  todo add a MCP3421 / MCP3424 18 Bit ADC

  ESP32 AZ-Delivery WROOM 32  V4

            5  C  D  D  1  G  1  1  2  2  2  3  3  3  3  V  V  E  3
            V  M  3  2  3  N  2  4  7  6  5  3  2  5  4  N  P  N  V 
               D            D                                      3
          --x--x--x--x--x--x--x--x--x--x--x--x--x--x--x--x--x--x--x-- 
         |                                                           +---+
         |                                                           |   |
         |+++++                                                      |   |
         |    |              AZ-Delivery ESP32 WROOM 32              |   |
         |    |                    Top-View                          |   |
         |+++++                                                      |   |
         |                                                           |   |
         |                                                           +---+
          --x--x--x--x--x--x--x--x--x--x--x--x--x--x--x--x--x--x--x--
            C  D  D  1  2  0  4  1  1  5  1  1  G  2  R  T  2  2  G
            L  0  1  5           6  7     8  9  N  1  X  X  2  3  N
            K                                   D                 D

  2.8Inch TFT SPI 320x240 V1.2 + resistive Touch + SD-Card Socket
  ILI9346 Driver
          
          +----------------------------------------------------------------------------+
          |                                                                            |
  T_IRQ   x            ESP32 ->   5                                                    |
  T_DO    x----+                                                                       |
  T_DIN   x----|---+                                                                   |
  T_CS    x    |   |   ESP32 ->  13                                                    |
  T_CLK   x -- |-+ |                                                                   |
          |    | | |                         Bottom View                               x  SD_SCK
  SDO     x----+ | |   ESP32 ->  23                                                    x  SD_MISO
  LED     x      | |   ESP32 -> 3V3 with 560 Ohm in Line                               x  SD_MOSI
  SCK     x------+ |   ESP32 ->  18                                                    x  SD_CS
  SDI     x--------+   ESP32 ->  23                                                    x  SD_CS
  DC      x            ESP32 ->  15                                                    |
  RESET   x            ESP32 ->  12                                                    |                                                            |
  CS      x            ESP32 ->  14                                                    |
  GND     x            ESP32 -> GND                                                    |
  VCC     x            ESP32 -> 3V3                                                    |
          |                                                                            |
          +----------------------------------------------------------------------------+


  Si5351 3 Channel PLL 
         
         +----------------+
         |   Top View     ++++ 
         |                ||||| SMA 0
     0   +                ++++
     1   +                |
     2   +                ++++        
   SCL   +  ESP32 ->  22  ||||| SMA 1    
   SDA   +  ESP32 ->  21  ++++ 
   GND   +  ESP32 -> GND  |
   VIN   +  ESP32 ->  5V  ++++ 
         |                ||||| SMA 2  
         |                ++++ 
         +----------------+    
 

  U-BLOX GPS NEO M7          

         +---------------------------+
         |          Top View         |
   VCC   +  ESP32 ->  5V             |
   GND   +  ESP32 -> GND             |
   RX    +  ESP32 ->  17             |
   TX    +  ESP32 ->  16             |
   PPS   +  ESP32 ->   4             |       
         |                           | 
         +---------------------------+    
   
 
*/

#include <Preferences.h>
Preferences preferences;
#include <EEPROM.h>
#define EEPROM_SIZE 60

// Touch
//#include "Adafruit_ILI9341.h"
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

// Initalize TFT
// Connections Display / Touch
#define TFT_RST   12
#define TOUCH_CS  13
#define TFT_CS    14
#define TFT_DC    15
#define SCLK_PIN  18
#define MISO_PIN  19
#define MOSI_PIN  23
#define TOUCH_IRQ  5

// Working with WaveShare 4 Inch Display
#include <ILI9486_SPI.h>   
ILI9486_SPI tft(TFT_CS, TFT_DC, TFT_RST);

#include "Tom_Button.h"
#include "Tom_Gps.h"
#include "Tom_Rotary.h"
#include "Tom_Cw.h"
#include "Tom_WSPR.h"

#include <WiFi.h>

//*******************************************************************************
// Hardware Serial Connection for GPS-Module
//*******************************************************************************
#include <HardwareSerial.h>
HardwareSerial SerialGps( 1 );
#define RXD2 16
#define TXD2 17

//*******************************************************************************
// Hardware Serial Connection for FT817
//*******************************************************************************
//#include <HardwareSerial.h>
HardwareSerial Serial817( 2 );
#define RXD3 26
#define TXD3 27



//*******************************************************************************
// GPS-Module
//*******************************************************************************
UBXMessage ubxMessage;

String myLocator   = "AAaaJJ";
int    msgType     = 0;

#define      ppsPin 4
volatile int ppsPulse = 0;
bool         GpsOk = false;

// Resetting GPS-Module to his factory default Settings
char UBLOX_M7_Set_Default_Config[] {
  0xB5,0x62,0x06,0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x03,0x1B,0x9A
};  

// Set GPS-Module to his working Conditions, no NMEA only UBX-Mode, 1Hz / 10Hz
char UBLOX_INIT[] {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off
  // Disable UBX
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off
  // Enable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1,   //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE,   //NAV-POSLLH  On
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5,   //NAV-STATUS  On
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x21,0x00,0x01,0x00,0x00,0x00,0x00,0x32,0x97,   //NAV-TIMEUTC On 

  // Rate 1Hz / 10Hz
  0xB5,0x62,0x06,0x31,0x20,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x0A,0x00,
  0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,0xD3,0x9E
};



//*******************************************************************************
// Rotary-Encoder
//*******************************************************************************
int        newRotAcc = 0;
// Remember previous state
static int oldRotAcc = 0; 

//*******************************************************************************
// Frequency
//*******************************************************************************
unsigned long frequencyA = 9000000;
unsigned long frequencyB = 9000000;
unsigned long frequencyC = 9000000;
// Step Management
int      StepNumA   = 11;
int      StepNumB   = 11;
int      StepNumC   = 11;
int      StepVal[12] {1, 10, 100, 1000, 5000, 9000, 10000, 12500, 25000, 50000, 100000, 1000000};
String   StepText[12] {"    1Hz", "   10Hz", "  100Hz", "   1KHz", "   5KHz" ,"   9KHz", "  10KHz" ,"12.5KHz", "  25KHz", "  50KHz", " 100KHz", "   1MHz"};

int     value       = 9000000;
// Channel 0, 1, 2
int     AktChannel = 0;  
int     PhaseA     = 0;
int     PhaseB     = 0;  
int     PhaseC     = 0;  
int     StrengthA  = 0; // 0=2MA, 1=4MA, 2=6MA, 3=8MA
int     StrengthB  = 0; // 0=2MA, 1=4MA, 2=6MA, 3=8MA
int     StrengthC  = 0; // 0=2MA, 1=4MA, 2=6MA, 3=8MA
String  StrAText[4] {"SA2", "SA4", "SA6", "SA8"};
String  StrBText[4] {"SB2", "SB4", "SB6", "SB8"};
String  StrCText[4] {"SC2", "SC4", "SC6", "SC8"};

bool    Aon        = false;
bool    Bon        = false;
bool    Con        = false;
bool    AgB        = false;

// PLL
#include <si5351.h>
#include "Wire.h"
Si5351 si5351;

//*******************************************************************************
// Touch-Screen
//*******************************************************************************
//XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
// Initialize Touch
XPT2046_Touchscreen ts(TOUCH_CS);  // Param 2 - Touch IRQ Pin - interrupt enabled polling

TS_Point p;
// Default-Settings Touch-Controller
int   tx_max  = 259;
int   ty_max  = 3862;
int   tx_min  = 3924;
int   ty_min  = 349;

bool  isTouched   = false;
bool  wastouched  = true;
bool  AutoRepeat  = false;

//*******************************************************************************
// CW
//*******************************************************************************
String cw_message  = "";
String cw_message1 = "VVV de DB1IAT/B = LOCATOR IS ";
String cw_message2 = " = PWR IS 10mW = ANT IS VERTICAL WIRE"; 

String EingabeText = "THOMAS";

#define pinBeep    2
#define myRotation 1

volatile int interruptCounter;
int totalInterruptCounter;
// Test IRQ

bool IrqColor = false;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;



//*****************************************************************
// format long variable with thousands seperator '.'
//*****************************************************************
char* ltoaKomma(long i){ 
#define LONG_MAXWIDTH 14  // −2.147.483.648 = 10 digits + sign + 3 dots
  static char buf[LONG_MAXWIDTH + 1];  // Room for formatted number  and '\0'
  char *p = buf + LONG_MAXWIDTH;       // points to terminating '\0'
  if (i<0) {
    buf[0]='-'; 
    i=-i;
  } else buf[0]='+';
  byte count=3;
  do {
    *--p = '0' + (i % 10);
    i /= 10;
    count--;
    if (count==0 && i>0) {
      *--p = '.';
      count=3;
    }
  } while (i!=0);
  if (buf[0]=='-') *--p = '-';
  return p;
}  

//-----------------------------------------------------------------------------------------
// Set Frequency of the Channel
//-----------------------------------------------------------------------------------------
bool SetChannelFrequency(int newFrequency) {
  bool isOk = false;
  if (newFrequency >= 100000) {
    if(newFrequency <= 170000000) {
      if (AktChannel == 0) {
        frequencyA = newFrequency;
        si5351.set_freq( frequencyA*SI5351_FREQ_MULT, SI5351_CLK0);
        isOk = true;
      }
      if (AktChannel == 1) {
        frequencyB = newFrequency;
        si5351.set_freq( frequencyB*SI5351_FREQ_MULT, SI5351_CLK1);
        isOk = true;
      }
      if (AktChannel == 2) {
        frequencyC = newFrequency;
        si5351.set_freq( frequencyC*SI5351_FREQ_MULT, SI5351_CLK2);
        isOk = true;
      }
      si5351.pll_reset(SI5351_PLLA);        
    }
  }
  return isOk;
}

//-----------------------------------------------------------------------------------------
// Frequency one Step up
//-----------------------------------------------------------------------------------------
void DoFrequnecyUp(void) {
  if (AktChannel == 0) {
    if (frequencyA+StepVal[StepNumA] <= 170000000) {
      frequencyA += StepVal[StepNumA];
      SetChannelFrequency(frequencyA);
      // Set Frequency A also to Channel B
      if (AgB) {frequencyB = frequencyA; si5351.set_freq(frequencyB*SI5351_FREQ_MULT, SI5351_CLK1);}
      si5351.pll_reset(SI5351_PLLA);        
    }
  }
  if (AktChannel == 1) {
    if (frequencyB+StepVal[StepNumB] <= 170000000) {
      frequencyB += StepVal[StepNumB];
      SetChannelFrequency(frequencyB);
      // Set Frequency B also to Channel A
      if (AgB) {frequencyA = frequencyB; si5351.set_freq(frequencyA*SI5351_FREQ_MULT, SI5351_CLK0);}
      si5351.pll_reset(SI5351_PLLA);        
    }
  }
  if (AktChannel == 2) {
    if (frequencyC+StepVal[StepNumC] <= 170000000) {
      frequencyC += StepVal[StepNumC];
      SetChannelFrequency(frequencyC);
    }
  }
}


//**********************************************************************
// PLL Init and set Correction
//**********************************************************************
void InitPll(void) {
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(94000, SI5351_PLL_INPUT_XO);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
}

unsigned long oTime = millis();
unsigned long mTime = millis();
int           DisplayMode = 0;

//**********************************************************************
//  Get some Memory for Canvas
//**********************************************************************
// Testing Canvas

//**********************************************************************
//  GPS Canvas
//**********************************************************************
void DrawGpsCanvas(void) {
  oTime = millis();
  uint8_t ox = 4;
  uint8_t oy = 4;
  GFXcanvas16 MyCanvas(140, 80); 
  
  MyCanvas.fillScreen(0);
  MyCanvas.drawRect(0, 0, MyCanvas.width(), MyCanvas.height(), DARKGREY);
  MyCanvas.setTextColor(WHITE, BLACK);
  
  MyCanvas.setTextSize(1);

  MyCanvas.setCursor(0+ox,0+oy);
  myLocator = compute_Locator(double(ubxMessage.NAV_PVT.lat/10000000.0), double(ubxMessage.NAV_PVT.lon/10000000.0));
  MyCanvas.print(myLocator);

  MyCanvas.setCursor(0+ox,26+oy);
  MyCanvas.print("Lat: "); MyCanvas.print(ubxMessage.NAV_PVT.lat/10000000.0, DEC);
  MyCanvas.setCursor(0+ox,36+oy);
  MyCanvas.print("Lon:  "); MyCanvas.print(ubxMessage.NAV_PVT.lon/10000000.0, DEC);
  MyCanvas.setCursor(85+ox, 0+oy);
  MyCanvas.print("SAT: ");     
  if (ubxMessage.NAV_PVT.num_sv < 10) MyCanvas.print("0");  
  MyCanvas.print(ubxMessage.NAV_PVT.num_sv);
  MyCanvas.setCursor(0+ox, 10+oy);
  if (ubxMessage.NAV_PVT.day < 10) MyCanvas.print("0");
  MyCanvas.print(ubxMessage.NAV_PVT.day); MyCanvas.print("."); 
  if (ubxMessage.NAV_PVT.month < 10) MyCanvas.print("0");
  MyCanvas.print(ubxMessage.NAV_PVT.month); MyCanvas.print("."); 
  MyCanvas.print(ubxMessage.NAV_PVT.year);
    
  MyCanvas.print(" / "); 
  if (ubxMessage.NAV_PVT.hour < 10) MyCanvas.print("0");
  MyCanvas.print(ubxMessage.NAV_PVT.hour); MyCanvas.print(":"); 
  if (ubxMessage.NAV_PVT.min < 10) MyCanvas.print("0");
  MyCanvas.print(ubxMessage.NAV_PVT.min); MyCanvas.print(":"); 
  if (ubxMessage.NAV_PVT.sec < 10) MyCanvas.print("0");
  MyCanvas.print(ubxMessage.NAV_PVT.sec); 
  MyCanvas.setCursor(0+ox,56+oy);
  MyCanvas.print("FIX: "); MyCanvas.print(ubxMessage.NAV_PVT.fix_type);
  MyCanvas.setCursor(66+ox,56+oy);
  MyCanvas.print("H_MSL:"); 
  if (ubxMessage.NAV_PVT.h_msl/1000 < 1000) MyCanvas.print(" ");
  if (ubxMessage.NAV_PVT.h_msl/1000 <  100) MyCanvas.print(" ");
  if (ubxMessage.NAV_PVT.h_msl/1000 <   10) MyCanvas.print(" ");
  if (ubxMessage.NAV_PVT.h_msl/1000 >= 0) {
    MyCanvas.print(ubxMessage.NAV_PVT.h_msl/1000);
  }
  MyCanvas.setCursor(66+ox,66+oy);
  MyCanvas.print("Head:  "); 
  if (ubxMessage.NAV_PVT.head_mot/100000 < 100) MyCanvas.print(" ");
  if (ubxMessage.NAV_PVT.head_mot/100000 <  10) MyCanvas.print(" ");
  if (ubxMessage.NAV_PVT.head_mot/100000 <= 360){
    MyCanvas.print(ubxMessage.NAV_PVT.head_mot/100000);
  }
  tft.drawRGBBitmap(333, 3, MyCanvas.getBuffer(), MyCanvas.width(), MyCanvas.height());
  Serial.print("DrawGpsCanvas "); Serial.print(millis() - oTime); Serial.println(" ms");
}

//**********************************************************************
//  PLL Canvas
//**********************************************************************
void DrawPllCanvas(void) {
  GFXcanvas16 MyCanvas(320, 80); 
  uint8_t ox = 8;
  uint8_t oy = 8;
  oTime = millis();
  MyCanvas.fillScreen(0);
  MyCanvas.drawRect(0, 0, MyCanvas.width(), MyCanvas.height(), DARKGREY);
  MyCanvas.setTextColor(WHITE, BLACK);
  MyCanvas.setTextSize(2);
  MyCanvas.setCursor(0+ox,0+oy);
  if (AktChannel == 0) {MyCanvas.setTextColor(GREEN, BLACK);} else {MyCanvas.setTextColor(WHITE, BLACK);}
  MyCanvas.print("FA "); 
  if (frequencyA < 100000000) MyCanvas.print(" ");
  if (frequencyA <  10000000) MyCanvas.print(" ");
  if (frequencyA <   1000000) MyCanvas.print("  ");
  MyCanvas.print(ltoaKomma(frequencyA));
  if (PhaseA > 0) {
    MyCanvas.setCursor(180, 0+oy); 
    MyCanvas.setTextSize(1); 
    MyCanvas.print(PhaseA); 
    if (AktChannel == 0) MyCanvas.drawCircle(198, 0+oy, 1, GREEN); else MyCanvas.drawCircle(198, 0+oy, 1, WHITE); 
    MyCanvas.setTextSize(2);
  }
  MyCanvas.setCursor(215+ox, 0+oy);
  MyCanvas.print(StepText[StepNumA]);
  if (Aon) {MyCanvas.fillRect(205, 11, 9, 9, DARKGREY); MyCanvas.fillRect(206, 12, 7, 7, RED);}

  MyCanvas.setCursor(0+ox,22+oy);
  MyCanvas.setTextColor(WHITE, BLACK);
  if (AktChannel == 1) {MyCanvas.setTextColor(GREEN, BLACK);} else {MyCanvas.setTextColor(WHITE, BLACK);}
  MyCanvas.print("FB "); 
  if (frequencyB < 100000000) MyCanvas.print(" ");
  if (frequencyB <  10000000) MyCanvas.print(" ");
  if (frequencyB <   1000000) MyCanvas.print("  ");
  MyCanvas.print(ltoaKomma(frequencyB));
  if (PhaseB > 0) {
    MyCanvas.setCursor(180, 22+oy); 
    MyCanvas.setTextSize(1); 
    MyCanvas.print(PhaseB); 
    if (AktChannel == 1) MyCanvas.drawCircle(198, 22+oy, 1, GREEN); else MyCanvas.drawCircle(198, 22+oy, 1, WHITE); 
    MyCanvas.setTextSize(2);
  }
  MyCanvas.setCursor(215+ox, 22+oy);
  MyCanvas.print(StepText[StepNumB]);
  if (Bon) {MyCanvas.fillRect(205, 33, 9, 9, DARKGREY); MyCanvas.fillRect(206, 34, 7, 7, RED);}

  MyCanvas.setCursor(0+ox,44+oy);
  MyCanvas.setTextColor(WHITE, BLACK);
  if (AktChannel == 2) {MyCanvas.setTextColor(GREEN, BLACK); } else {MyCanvas.setTextColor(WHITE, BLACK);}
  MyCanvas.print("FC "); 
  if (frequencyC < 100000000) MyCanvas.print(" ");
  if (frequencyC <  10000000) MyCanvas.print(" ");
  if (frequencyC <   1000000) MyCanvas.print("  ");
  MyCanvas.print(ltoaKomma(frequencyC));
  if (PhaseC > 0) {
    MyCanvas.setCursor(180, 44+oy); 
    MyCanvas.setTextSize(1); 
    MyCanvas.print(PhaseC); 
    if (AktChannel == 2) MyCanvas.drawCircle(198, 44+oy, 1, GREEN); else MyCanvas.drawCircle(198, 44+oy, 1, WHITE); 
    MyCanvas.setTextSize(2);
  }
  MyCanvas.setCursor(215+ox, 44+oy);
  MyCanvas.print(StepText[StepNumC]);
  if (Con) {MyCanvas.fillRect(205, 55, 9, 9, DARKGREY); MyCanvas.fillRect(206, 56, 7, 7, RED);}

  //MyPllCanvas.setTextColor(WHITE, BLACK);

  tft.drawRGBBitmap(6, 3, MyCanvas.getBuffer(), MyCanvas.width(), MyCanvas.height());
  Serial.print("DrawPllCanvas "); Serial.print(millis() - oTime); Serial.println(" ms");
}


unsigned long ppsTime     = millis();
volatile int  ppsDuration = 0;

//**********************************************************************
//  IRQ PPS
//**********************************************************************
void ISRpps(void) {
  if (ppsPulse == 0) ppsTime = millis();
  ppsPulse ++;
  if (ppsPulse == 11) {
    ppsPulse = 0;
    ppsDuration = millis() - ppsTime;
  }
}


//-----------------------------------------------------------------------------------------
//  Update Display
//-----------------------------------------------------------------------------------------
void UpdateDisplay(void) {
  Serial.println("UpdateDisplay");
  //tft.drawRect(0,0,320, 240, DARKGREY);
  if (DisplayMode == 0) {
    DrawPllCanvas();
    DrawGpsCanvas();
  }
  if (DisplayMode == 1) {
    DrawGpsCanvas();
  }
}


//******************************************************************************
// Initialize Menu Number 1
//******************************************************************************
void InitMenu1(void) {
  int BtnWidth = 78;
  int BtnHeight = 38;
  Serial.println("DrawMenu1");
  tft.fillScreen(LIGHTGREY);
  ClearButtons();
  int  btn = 0;
  int  state = -1;
  bool OnOff = false;
  // Fill Buttons
  for(int x=0; x<6; x++) {
    for(int i=0; i<6; i++) {
      btn = i+(x*6);
      String s = String(btn);
      if (btn ==  0) {
        OnOff = true;
        s = "SelA";
        if (AktChannel == 0) state = 1;
      }
      if (btn ==  1) {
        OnOff = true;
        s = "SelB"; 
        if (AktChannel == 1) state = 1;
      }
      if (btn ==  2) {
        OnOff = true;
        s = "SelC"; 
        if (AktChannel == 2) state = 1;
      }
      if (btn ==  3) { s = "PLL"; if (DisplayMode == 0) state =1; OnOff = true;}
      if (btn ==  6) {s = StrAText[StrengthA];}
      if (btn ==  7) {s = StrBText[StrengthB];}
      if (btn ==  8) {s = StrCText[StrengthC];}
      if (btn ==  9) { s = "GPS"; if (DisplayMode == 1) state =1; OnOff = true;}
      if (btn == 12) { s = "Aon"; if (Aon) state = 1; OnOff = true; }
      if (btn == 13) { s = "Bon"; if (Bon) state = 1; OnOff = true;}
      if (btn == 14) { s = "Con"; if (Con) state = 1; OnOff = true;}
      if (btn == 15) s = "CAL";
      if (btn == 18) {s = "F UP"; }
      if (btn == 19) {s = "F DWN";}
      if (btn == 20) {s = "STEP";}
      if (btn == 21) { s = "A=B"; if (AgB) state = 1; OnOff = true;}
      if (btn == 24) {s = "CW"; OnOff = true; }
      if (btn == 25) s = "NCW";
      if (btn == 26) s = "PH";
      if (btn == 27) s = "INPA";
      if (btn == 30) s = "INPN";
      if (btn == 31) s = "STORE";
      if (btn == 32) s = "LOAD";
      if (btn == 33) s = "RESET";
      if (ButtonAdd(6+x*BtnWidth, 86+i*BtnHeight, BtnWidth-2, BtnHeight-2, state, OnOff, true, s, LIGHTGREY, BLACK)) {
        //Serial.print("Button Added "); Serial.println(s);
      } else {
        Serial.println("Error -> Button Not Added, nomore Space"); 
      }
      state = -1;
      OnOff = false;
      // if there is no Button Text eg. only the ButtonNumber set the Button to Invisible
      //if (GetButtonText(btn) == String(btn)) {
      //  SetButtonVisibility(btn, false);
      //}
    }
  }  
  
  // Invisible Buttons for Changing Channels
  ButtonAdd(4,  5, 310, 20, state, OnOff, false, "36", LIGHTGREY, BLACK);
  ButtonAdd(4, 27, 310, 20, state, OnOff, false, "37", LIGHTGREY, BLACK);
  ButtonAdd(4, 49, 310, 20, state, OnOff, false, "38", LIGHTGREY, BLACK);

  //SetOnClick(0, &onClickFunction);
  SetAutoRepeat(18, true);
  SetAutoRepeat(19, true);
  SetAutoRepeat(20, true);
  tone(pinBeep, 1000, 250);
  DrawButtons(-1);
  UpdateDisplay();
}



//******************************************************************************
// Update Menu Number 1
//******************************************************************************
void UpdateMenu1(void) {
  Serial.println("Update Menu1");
  if (AktChannel == 0) {
     SetButtonState(0, 1);
     SetButtonState(1, 0);
     SetButtonState(2, 0);
  }  
  if (AktChannel == 1) {
     SetButtonState(0, 0);
     SetButtonState(1, 1);
     SetButtonState(2, 0);
  }  
  if (AktChannel == 2) {
     SetButtonState(0, 0);
     SetButtonState(1, 0);
     SetButtonState(2, 1);
  }  
  if (DisplayMode == 0) {
     SetButtonState(4, 1);
     SetButtonState(5, 0);
  }   
  if (DisplayMode == 1) {
     SetButtonState(4, 0);
     SetButtonState(5, 1);
  }   
  SetButtonText(6, StrAText[StrengthA]);
  SetButtonText(7, StrAText[StrengthB]);
  SetButtonText(8, StrAText[StrengthC]);
  if (AgB) SetButtonState(16, 1); else SetButtonState(16, 0);
  if (Aon) SetButtonState(12, 1); else SetButtonState(12, 0);
  if (Bon) SetButtonState(13, 1); else SetButtonState(13, 0);
  if (Con) SetButtonState(14, 1); else SetButtonState(14, 0);
}


//**********************************************************************
//  Store default Settings
//**********************************************************************
void StoreSettings(void) {
  preferences.begin("Parfile", false);
  preferences.putBool("PllAon", Aon);
  preferences.putBool("PllBon", Bon);
  preferences.putBool("PllCon", Con);
  preferences.putULong("FreqA", frequencyA);
  preferences.putULong("FreqB", frequencyB);
  preferences.putULong("FreqC", frequencyC);
  preferences.putInt("DisplayMode", DisplayMode);
  preferences.putInt("AktChannel", AktChannel);
  preferences.putInt("StepNumA", StepNumA);
  preferences.putInt("StepNumB", StepNumB);
  preferences.putInt("StepNumC", StepNumC);
  preferences.putInt("StengthA", StrengthA);
  preferences.putInt("StengthB", StrengthB);
  preferences.putInt("StengthC", StrengthC);
  preferences.putInt("PhaseA",   PhaseA);
  preferences.putInt("PhaseB",   PhaseB);
  preferences.putInt("PhaseC",   PhaseC);
  preferences.end();  
}

//**********************************************************************
//  Load default Settings
//**********************************************************************
void LoadSettings(void) {
  preferences.begin("Parfile", false);
  Aon = preferences.getBool("PllAon", false);
  Bon = preferences.getBool("PllBon", false);
  Con = preferences.getBool("PllCon", false);
  frequencyA  = preferences.getULong("FreqA", 9000000);
  frequencyB  = preferences.getULong("FreqB", 9000000);
  frequencyC  = preferences.getULong("FreqC", 9000000);
  DisplayMode = preferences.getInt("DisplayMode", 0);
  AktChannel  = preferences.getInt("AktChannel", 0);
  if (AktChannel  == 0) {SetButtonState(0, 1); SetButtonState(1, 0); SetButtonState(2, 0);}
  if (AktChannel  == 1) {SetButtonState(0, 0); SetButtonState(1, 1); SetButtonState(2, 0);}
  if (AktChannel  == 2) {SetButtonState(0, 0); SetButtonState(1, 0); SetButtonState(2, 1);}
  if (DisplayMode == 0) {SetButtonState(4, 1); SetButtonState(5, 0);}
  if (DisplayMode == 1) {SetButtonState(5, 1); SetButtonState(4, 0);}
  if (Aon) SetButtonState(12, 1); else SetButtonState(12, 0); 
  if (Bon) SetButtonState(13, 1); else SetButtonState(13, 0);
  if (Con) SetButtonState(14, 1); else SetButtonState(14, 0);
  StepNumA  = preferences.getInt("StepNumA", 0);
  StepNumB  = preferences.getInt("StepNumB", 0);
  StepNumC  = preferences.getInt("StepNumC", 0);
  StrengthA = preferences.getInt("StengthA", 0);
  StrengthB = preferences.getInt("StengthB", 0);
  StrengthC = preferences.getInt("StengthC", 0);
  PhaseA    = preferences.getInt("PhaseA", 0);
  PhaseB    = preferences.getInt("PhaseB", 0);
  PhaseC    = preferences.getInt("PhaseC", 0);
  preferences.end();  
  UpdateMenu1();
  //DrawButtons(&tft, -1);
  //UpdateDisplay();
}


callback_funct onClickFunction; // variable to store function pointer type

void Button0onClick(void) {
  Serial.println("Button 0 Clicked");
}


void ScanWiFiNetwork(void) {
  Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0) {
      Serial.println("no networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
    }
  }
  Serial.println("");
}


//-----------------------------------------------------------------------------------------
// The Rotary Encoder is Pushed
//-----------------------------------------------------------------------------------------
void DoRotaryPush(void) {
  if (digitalRead(pushPin) == LOW) {
    while (digitalRead(pushPin) == LOW) {
      delay(2); 
    }
    if (AktChannel == 0) {
      StepNumA ++;
      if (StepNumA > 11) StepNumA = 0;
      Serial.print("StepwidthA "); Serial.println(StepVal[StepNumA]);
    }
    if (AktChannel == 1) {
      StepNumB ++;
      if (StepNumB > 11) StepNumB = 0;
      Serial.print("StepwidthB "); Serial.println(StepVal[StepNumB]);
    }
    if (AktChannel == 2) {
      StepNumC ++;
      if (StepNumC > 11) StepNumC = 0;
      Serial.print("StepwidthC "); Serial.println(StepVal[StepNumC]);
    }
    //UpdateDisplay();
    if (DisplayMode == 0) {
      DrawPllCanvas();
    }
  }
}

//-----------------------------------------------------------------------------------------
// Do the GPS Stuff
//-----------------------------------------------------------------------------------------
void DoGPS(void) {
  msgType = processGPS(&SerialGps, &ubxMessage);
  if ( msgType == MT_NAV_PVT ) {
    //Serial.println("MT_NAV_PVT");
  }
}

//-----------------------------------------------------------------------------------------
// Frequency one Step down
//-----------------------------------------------------------------------------------------
void DoFrequnecyDown(void) {
  if (AktChannel == 0) {
    if (frequencyA-StepVal[StepNumA] >= 100000) {
      frequencyA -= StepVal[StepNumA];
      SetChannelFrequency(frequencyA);
      // Set Frequency A also to Channel B
      if (AgB) {frequencyB = frequencyA; si5351.set_freq(frequencyB*SI5351_FREQ_MULT, SI5351_CLK1);}
      si5351.pll_reset(SI5351_PLLA);        
    }  
  }  
  if (AktChannel == 1) {
    if (frequencyB-StepVal[StepNumB] >= 100000) {
      frequencyB -= StepVal[StepNumB];
      SetChannelFrequency(frequencyB);
      // Set Frequency B also to Channel A
      if (AgB) { frequencyA = frequencyB; si5351.set_freq(frequencyA*SI5351_FREQ_MULT, SI5351_CLK0);}
      si5351.pll_reset(SI5351_PLLA);        
    }  
  }  
  if (AktChannel == 2) {
    if (frequencyC-StepVal[StepNumC] >= 100000) {
      frequencyC -= StepVal[StepNumC];
      SetChannelFrequency(frequencyC);
    }  
  }  
  if (frequencyA < 100000) frequencyA = 100000;
  if (frequencyB < 100000) frequencyB = 100000;
  if (frequencyC < 100000) frequencyC = 100000;
}

//********************************************************************
// Rotary Encoder Changed
//********************************************************************
bool DoRotaryChanged() {
  bool changed = false;
  newRotAcc = GetRotAcc();
  if (newRotAcc > oldRotAcc) { 
    // Right Turn
    DoFrequnecyUp();
    changed = true;
    //Serial.println(frequency);
  } else   
  if (newRotAcc < oldRotAcc) { 
    // Left Turn
    DoFrequnecyDown();
    changed = true;
    //Serial.println(frequency);
  }       
  oldRotAcc = newRotAcc;
  return changed;
}

bool firstClick = true;

//*************************************************************************************
// Check wich Button is Touched
//*************************************************************************************
int CheckTouchedButton(void) {
  int s = -1;
  bool Repeat = false;
  if ((ts.touched()) && (!isTouched)) {
    p = ts.getPoint();
    //Serial.print("RAW Touched at "); Serial.print(p.x); Serial.print(" "); Serial.print(p.y); Serial.print(" "); Serial.println(p.z);
    p.x = map(p.x, tx_min, tx_max, 0, 480);
    p.y = map(p.y, ty_min, ty_max, 0, 320);
    if (p.x > 480) p.x = 480;
    if (p.y > 320) p.y = 320;
    //tft.drawPixel(p.x, p.y, WHITE);
    //Serial.print("Touched at "); Serial.print(p.x); Serial.print(" "); Serial.print(p.y); Serial.print(" "); Serial.println(p.z);
    tone(pinBeep, 1200, 6);
    if (p.z > 300) {
      s = GetButtonByPos(p.x, p.y);
      // If ther was a Button Pressed
      if (s > -1) {
        Repeat = GetAutoRepeat(s);
        // Draw the pressed Button as down
        SetButtonState(s, 1); 
        //DrawPllCanvas();
        if ((firstClick) && (Repeat == true)) delay(250);
        firstClick = false;
      }
    }
    // Wait until the Button is released
    if (!Repeat) while (ts.touched());
    // DrawButton as up
    if (!GetButtonOnOff(s)) SetButtonState(s, 0);
  }
  if (!Repeat) while (ts.touched());
  if (!ts.touched()) firstClick = true;
  delay(50);
  return s;
}


// *************************************************************
//  Show Input Box / Keyboard and read the new Value
// *************************************************************
int InputBoxNum(int value, String Caption) {
  char v[16][5] ={"'*", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", ".", "<", "Clr", "OK", "Exit"};
  int i = 0;
  int x = 60;
  int y = 60;
  int w = 320;
  int h = 165;
  // Holds the Value to Display
  String s = String(value);
  // Startpoint and Size of the InputBox
  int sx = 70;
  int sy = 130;
  int kw = 57;
  int kh = 28;
  // Did we receive a OK from the User   
  boolean myOk =false;
  // Caption
  //tft.fillScreen(0);
  tft.drawRect(x-1, y-1, w+1, h+1, DARKGREY);
  tft.drawFastHLine(x-1,  y-1, w+1, WHITE);
  tft.drawFastVLine(x-1,  y, h, WHITE);
  tft.fillRect(x, y, 319, 22, BLUE);
  tft.setTextColor(YELLOW, BLUE);
  tft.setTextSize(2);
  tft.setCursor(x+4, y+4); 
  tft.print(Caption); 
  // Canvas 
  tft.fillRect(x, y+22, 319, 164-22, LIGHTGREY);
  // Input Edit
  tft.fillRect(x+10, y+31, 180, 30, BLACK);
  tft.drawFastHLine(x+11,   y+32, 180, LIGHTGREY);
  tft.drawFastVLine(x+11,   y+32,  30, LIGHTGREY);
  tft.drawFastHLine(x+11,   y+62, 180, WHITE);
  tft.drawFastVLine(x+191,  y+32,  30, WHITE);
  tft.setTextColor(GREEN, BLACK);
  tft.setTextSize(2);
  tft.setCursor(x+20, y+40); 
  tft.print(s); 
  
  // Keys
  ClearButtons();
  tft.setTextColor(YELLOW, BLUE);
  tft.setTextSize(2);
  for (int y=0; y<3; y++) {
    for (int x=0; x<5; x++) {
      ButtonAdd(sx+(kw+1)*x, sy+(kh+1)*y, kw, kh, 0, false, true, v[i+1], BLUE, YELLOW);
      i++;
    }
  }
  DrawButtons(-1);
  
  tft.setTextColor(YELLOW, BLUE);
  tft.setTextSize(2);
  // Handle touch
  while (1) {
    int cPos = CheckTouchedButton();
    if (cPos > -1) {
      if (cPos <= 10) {
        if (s.length() < 10) s += String(v[cPos+1]);
      } 
      if (cPos == 11) {
          s.remove(s.length()-1);
      }
      if (cPos == 12) {
        s = "";
      }
      if (cPos == 13) {
        myOk = true;
        break;
      }
      if (cPos == 14) {
        myOk = false;
        break;
      }
      tft.fillRect(x+10, y+31, 180, 30, BLACK);
      tft.drawFastHLine(x+11,   y+32, 180, LIGHTGREY);
      tft.drawFastVLine(x+11,   y+32,  30, LIGHTGREY);
      tft.drawFastHLine(x+11,   y+62, 180, WHITE);
      tft.drawFastVLine(x+191,  y+32,  30, WHITE);
      tft.setTextColor(GREEN, BLACK);
      tft.setTextSize(2);
      tft.setCursor(x+20, y+40); 
      tft.print(s); 
      tft.setTextColor(YELLOW, BLUE);
    }
  }
  if (myOk == true) return s.toInt();
    else return value;
}

// *************************************************************
//  Show Input Box / Keyboard and read the new Value
// *************************************************************
String InputBoxAlp(String value, String Caption) {
  char v[44][5] ={"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", 
                  "Q", "W", "E", "R", "T", "Z", "U", "I", "O", "P",
                  "A", "S", "D", "F", "G", "H", "J", "K", "L", " ",
                  "Y", "X", "C", "V", "B", "N", "M", ",", ".", "-",
                  "<", "CLR", "OK", "EXIT"};
  int i = 0;
  int x = 60;
  int y = 60;
  int w = 320;
  int h = 220;
  // Holds the Value to Display
  String s = String(value);
  // Startpoint and Size of the InputBox
  int sx = 70;
  int sy = 130;
  int kw = 28;
  int kh = 28;
  // Did we receive a OK from the User   
  boolean myOk = false;
  // Caption
  //tft.fillScreen(0);
  tft.drawRect(x-1, y-1, w+1, h+1, DARKGREY);
  tft.drawFastHLine(x-1,  y-1, w+1, WHITE);
  tft.drawFastVLine(x-1,  y, h, WHITE);
  tft.fillRect(x, y, 319, 22, BLUE);
  tft.setTextColor(YELLOW, BLUE);
  tft.setTextSize(2);
  tft.setCursor(x+4, y+4); 
  tft.print(Caption); 
  // Canvas 
  tft.fillRect(x, y+22, 319, h-23, LIGHTGREY);
  // Input Edit
  tft.fillRect(x+10, y+31, 220, 30, BLACK);
  tft.drawFastHLine(x+10,   y+30, 220, LIGHTGREY);
  tft.drawFastVLine(x+10,   y+30,  30, LIGHTGREY);
  tft.drawFastHLine(x+10,   y+62, 220, WHITE);
  tft.drawFastVLine(x+230,  y+32,  30, WHITE);
  tft.setTextColor(GREEN, BLACK);
  tft.setTextSize(2);
  tft.setCursor(x+20, y+40); 
  tft.print(s); 
  
  // Keys
  ClearButtons();
  tft.setTextColor(YELLOW, BLUE);
  tft.setTextSize(2);
  for (int y=0; y<4; y++) {
    for (int x=0; x<10; x++) {
      ButtonAdd(sx+(kw+1)*x, sy+(kh+1)*y, kw, kh, 0, false, true, v[i], BLUE, YELLOW);
      i++;
    }
  }
  ButtonAdd(sx, 247, kw, kh, 0, false, true, v[40], BLUE, YELLOW);
  ButtonAdd(sx+kw+2, 247, 55, kh, 0, false, true, v[41], BLUE, YELLOW);
  ButtonAdd(sx+86, 247, 57, kh, 0, false, true, v[42], BLUE, YELLOW);
  ButtonAdd(sx+145, 247, 57, kh, 0, false, true, v[43], BLUE, YELLOW);
  
  DrawButtons(-1);
  
  tft.setTextColor(YELLOW, BLUE);
  tft.setTextSize(2);
  // Handle touch
  while (1) {
    int cPos = CheckTouchedButton();
    if (cPos > -1) {
      if (cPos <= 39) {
        if (s.length() < 17) s += String(v[cPos]);
      } 
      if (cPos == 40) {
          s.remove(s.length()-1);
      }
      if (cPos == 41) {
        s = "";
      }
      if (cPos == 42) {
        myOk = true;
        break;
      }
      if (cPos == 43) {
        myOk = false;
        break;
      }
      tft.fillRect(x+10, y+31, 220, 30, BLACK);
      tft.drawFastHLine(x+10,   y+30, 220, LIGHTGREY);
      tft.drawFastVLine(x+10,   y+30,  30, LIGHTGREY);
      tft.drawFastHLine(x+10,   y+62, 220, WHITE);
      tft.drawFastVLine(x+230,  y+32,  30, WHITE);
      tft.setTextColor(GREEN, BLACK);
      tft.setTextSize(2);
      tft.setCursor(x+20, y+40); 
      tft.print(s); 
      tft.setTextColor(YELLOW, BLUE);
    }
  }
  if (myOk == true) return s;
    else return value;
}

//*************************************************************************************
//  Try to Map the Touch results
//*************************************************************************************
void MapTouchScreen(void) {
  int   tx[4];
  int   ty[4];

  tft.fillScreen(BLACK);
  tft.setTextColor(YELLOW, BLACK);
  tft.setRotation(myRotation);
  tft.setTextSize(1);
  
  tft.fillRect(0,   0, 2, 2, WHITE);
  tft.setCursor(80,  120); tft.print("Touch left top"); 
  // Wait for touch
  while (!ts.touched()) {}
  p = ts.getPoint();
  tx[0] = p.x;
  ty[0] = p.y;
  tft.setCursor(20,  40); tft.print("X "); tft.print(tx[0]); 
  tft.setCursor(20,  60); tft.print("Y "); tft.print(ty[0]); 
  // Wait for release
  while (ts.touched()) {}
  delay(1000);
  
  tft.fillRect(478,   0, 2, 2, WHITE);
  tft.setCursor(80,  120); tft.print("Touch right top"); 
  // Wait for touch
  while (!ts.touched()) {}
  p = ts.getPoint();
  tx[1] = p.x;
  ty[1] = p.y;
  tft.setCursor(270,  40); tft.print("X "); tft.print(tx[1]); 
  tft.setCursor(270,  60); tft.print("Y "); tft.print(ty[1]); 
  //tft.setCursor(140,  40); tft.print("tx_min1 "); tft.print(tx_min1); 
  //tft.setCursor(140,  60); tft.print("ty_min1 "); tft.print(ty_min1); 
  // Wait for release
  while (ts.touched()) {}
  delay(1000);
  
  tft.fillRect(0,   318, 2, 2, WHITE);
  tft.setCursor(80,  120); tft.print("Touch left bottom"); 
  // Wait for touch
  while (!ts.touched()) {}
  p = ts.getPoint();
  tx[2] = p.x;
  ty[2] = p.y;
  tft.setCursor(20,  160); tft.print("X "); tft.print(tx[2]); 
  tft.setCursor(20,  180); tft.print("Y "); tft.print(ty[2]); 
  // Wait for release
  while (ts.touched()) {}
  delay(1000);

  tft.fillRect(478,   318, 2, 2, WHITE);
  tft.setCursor(80,  120); tft.print("Touch right bottom"); 
  // Wait for touch
  while (!ts.touched()) {}
  p = ts.getPoint();
  tx[3] = p.x;
  ty[3] = p.y;
  tft.setCursor(270,  160); tft.print("X "); tft.print(tx[3]); 
  tft.setCursor(270,  180); tft.print("Y "); tft.print(ty[3]); 
  //tft.setCursor(140,  160); tft.print("tx_max1 "); tft.print(tx_max1); 
  //tft.setCursor(140,  180); tft.print("ty_max1 "); tft.print(ty_max1); 
  // Wait for release
  while (ts.touched()) {}
  delay(1000);
  // Calculate
  
  tx_min  = (tx[0] + tx[2]) / 2;  // 3922;
  ty_min  = (ty[0] + ty[1]) / 2;  // 3796;
  tx_max  = (tx[1] + tx[3]) / 2;  //  393;
  ty_max  = (ty[2] + ty[3]) / 2;  //  343;

  Serial.print("tx_min = "); Serial.println(tx_min); 
  Serial.print("ty_min = "); Serial.println(ty_min); 
  Serial.print("tx_max = "); Serial.println(tx_max); 
  Serial.print("ty_max = "); Serial.println(ty_max); 
  
  tft.setCursor(80,  120); tft.print("Touch somewhere to finish"); 
  while (ts.touched()) {}
  while (!ts.touched()) {}
  tft.fillScreen(BLACK);
  UpdateDisplay();
}


//-----------------------------------------------------------------------------------------
// Do the TouchScreen Stuff
//-----------------------------------------------------------------------------------------
void DoButtons(void) {
  int btn = CheckTouchedButton();
  if (btn > -1) {
    //Serial.println(btn);
    if (btn == 0) {
      Serial.println("Button 0 pressed");
      AktChannel = 0;
      SetButtonState(0, 1);
      SetButtonState(1, 0);
      SetButtonState(2, 0);
      if (DisplayMode == 0) DrawPllCanvas();
      //Serial.println("Channel A");
    }
    if (btn == 1) {
      AktChannel = 1;
      SetButtonState(1, 1);
      SetButtonState(0, 0);
      SetButtonState(2, 0);
      if (DisplayMode == 0) DrawPllCanvas();
      //Serial.println("Channel B");
    }
    if (btn == 2) {
      AktChannel = 2;
      SetButtonState(2, 1);
      SetButtonState(0, 0);
      SetButtonState(1, 0);
      if (DisplayMode == 0) DrawPllCanvas();
      //Serial.println("Channel B");
    }
    if (btn == 3) {
      SetButtonState(3, 1);
      SetButtonState(9, 0);
      DisplayMode = 0;
      UpdateDisplay();
    }  
    if (btn == 6) {
      StrengthA++;
      if (StrengthA > 3) StrengthA = 0;
      //Serial.print("StrengthA "); Serial.println(StrengthA);
      switch (StrengthA) {
        case 0: {si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); SetButtonText(btn, "SA2"); break; }
        case 1: {si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_4MA); SetButtonText(btn, "SA4"); break; }
        case 2: {si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA); SetButtonText(btn, "SA6"); break; }
        case 3: {si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); SetButtonText(btn, "SA8"); break; }
      }
      DrawButtonOnState(btn);
    }
    if (btn == 7) {
      StrengthB++;
      if (StrengthB > 3) StrengthB = 0;
      //Serial.print("StrengthB "); Serial.println(StrengthB);
      switch (StrengthB) {
        case 0: {si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); SetButtonText(btn, "SB2"); break; }
        case 1: {si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_4MA); SetButtonText(btn, "SB4"); break; }
        case 2: {si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA); SetButtonText(btn, "SB6"); break; }
        case 3: {si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA); SetButtonText(btn, "SB8"); break; }
      }
      DrawButtonOnState(btn);
    }
    if (btn == 8) {
      StrengthC++;
      if (StrengthC > 3) StrengthC = 0;
      //Serial.print("StrengthC "); Serial.println(StrengthC);
      switch (StrengthC) {
        case 0: {si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); SetButtonText(btn, "SC2"); break; }
        case 1: {si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_4MA); SetButtonText(btn, "SC4"); break; }
        case 2: {si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_6MA); SetButtonText(btn, "SC6"); break; }
        case 3: {si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA); SetButtonText(btn, "SC8"); break; }
      }
      DrawButtonOnState(btn);
    }
    if (btn == 9) {
      SetButtonState(9, 1);
      SetButtonState(3, 0);
      DisplayMode = 1;
      UpdateDisplay();
    }  
    if (btn == 12) {
      Aon = !Aon;
      if (Aon) { 
        si5351.output_enable(SI5351_CLK0, 1); 
        SetButtonState(btn, 1); 
      } else {
        si5351.output_enable(SI5351_CLK0, 0);
        SetButtonState(btn, 0); 
      }
      DrawPllCanvas();
    }
    if (btn == 13) {
      Bon = !Bon;
      if (Bon) {
        si5351.output_enable(SI5351_CLK1, 1); 
        SetButtonState(btn, 1); 
      } else {
        si5351.output_enable(SI5351_CLK1, 0);
        SetButtonState(btn, 0); 
      }
      DrawPllCanvas();
    }
    if (btn == 14) {
      Con = !Con;
      if (Con) {
        si5351.output_enable(SI5351_CLK2, 1); 
        SetButtonState(btn, 1); 
      } else {
        si5351.output_enable(SI5351_CLK2, 0);
        SetButtonState(btn, 0); 
      }
      DrawPllCanvas();
    }
    if (btn == 15) {
      MapTouchScreen();
    }
    if (btn == 18) {
      DoFrequnecyUp();
      UpdateDisplay();
      delay(5);
    }
    if (btn == 19) {
      DoFrequnecyDown();
      UpdateDisplay();
      delay(5);
    }
    if (btn == 20) {
      if (AktChannel == 0) {
        StepNumA ++;
        if (StepNumA > 11) StepNumA = 0;
        Serial.print("StepwidthA "); Serial.println(StepVal[StepNumA]);
      }
      if (AktChannel == 1) {
        StepNumB ++;
        if (StepNumB > 11) StepNumB = 0;
        Serial.print("StepwidthB "); Serial.println(StepVal[StepNumB]);
      }
      if (AktChannel == 2) {
        StepNumC ++;
        if (StepNumC > 11) StepNumC = 0;
        Serial.print("StepwidthC "); Serial.println(StepVal[StepNumC]);
      }
      //UpdateDisplay();
      if (DisplayMode == 0) {
        DrawPllCanvas();
      }
      delay(50);
    }
    if (btn == 21) {
      AgB = !AgB;
      if (AgB) {SetButtonState(btn, 1); } else {SetButtonState(btn, 0);}
    }
    if (btn == 24) {
      SetButtonState(24, 1);
      DrawButtons(-1);
      cw_message = cw_message1 + myLocator + cw_message2;
      DoCw(cw_message, pinBeep, &si5351); 
      SetButtonState(24, 0);
      DrawButtons(-1);
    }
    if (btn == 26) {
      if (AktChannel == 0) {
        PhaseA = InputBoxNum(PhaseA, "Channel A PHASE");
        si5351.set_phase(SI5351_CLK0, PhaseA);
        si5351.pll_reset(SI5351_PLLA);        
        Serial.print("Phase A = "); Serial.println(PhaseA);
      } else
      if (AktChannel == 1) {
        PhaseB = InputBoxNum(PhaseB, "Channel B PHASE");
        si5351.set_phase(SI5351_CLK1, PhaseB);
        si5351.pll_reset(SI5351_PLLA);        
        Serial.print("Phase B = "); Serial.println(PhaseB);
      }
      if (AktChannel == 2) {
        PhaseC = InputBoxNum(PhaseC, "Channel C PHASE");
        si5351.set_phase(SI5351_CLK2, PhaseC);
        si5351.pll_reset(SI5351_PLLA);        
        Serial.print("Phase C = "); Serial.println(PhaseC);
      }
      InitMenu1();
      UpdateDisplay();
    }
    if (btn == 27) {
      EingabeText = InputBoxAlp(EingabeText, "Texteingabe");
      InitMenu1();
      UpdateDisplay();
    }
    if (btn == 30) {
      if (AktChannel == 0) {
        value = InputBoxNum(frequencyA, "Channel A Frequency");
        if (!SetChannelFrequency(value)) {
          tone(pinBeep, 500, 500);
        }
        InitMenu1();
        UpdateDisplay();
      }
      if (AktChannel == 1) {
        value = InputBoxNum(frequencyB, "Channel B Frequency ");
        if (!SetChannelFrequency(value)) {
          tone(pinBeep, 500, 500);
        }
        InitMenu1();
        UpdateDisplay();
      }
      if (AktChannel == 2) {
        value = InputBoxNum(frequencyC, "Channel C Frequency");
        if (!SetChannelFrequency(value)) {
          tone(pinBeep, 500, 500);
        }
        InitMenu1();
        UpdateDisplay();
      }
    }   
    if (btn == 31) {
      StoreSettings();
      UpdateDisplay();
    }
    if (btn == 32) {
      LoadSettings();
      UpdateDisplay();
    }
    if (btn == 33) {
      ESP.restart();
    }
    if (btn == 36) {
      AktChannel = 0;
      SetButtonState(0, 1);
      SetButtonState(1, 0);
      SetButtonState(2, 0);
      if (DisplayMode == 0) DrawPllCanvas();
    }
    if (btn == 37) {
      AktChannel = 1;
      SetButtonState(0, 0);
      SetButtonState(1, 1);
      SetButtonState(2, 0);
      if (DisplayMode == 0) DrawPllCanvas();
    }
    if (btn == 38) {
      AktChannel = 2;
      SetButtonState(0, 0);
      SetButtonState(1, 0);
      SetButtonState(2, 1);
      if (DisplayMode == 0) DrawPllCanvas();
    }
  }
}


//*******************************************************************************
// On Timer IRQ
//*******************************************************************************
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  // Get GPS Data form SerialGps
  DoGPS();
  portEXIT_CRITICAL_ISR(&timerMux);
}


//******************************************************************************
// S E T U P
//******************************************************************************
void setup() {
  // SD-Card CS   
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  // Background LED Light
  analogWrite(2, 255);
  //WiFi Activation
  //WiFi.mode(WIFI_STA);
  //WiFi.disconnect();

  
  tone(pinBeep, 2000, 50);
  //Init EEPROM
  EEPROM.begin(EEPROM_SIZE);
  //preferences.clear();
  
  Serial.begin(115200);
  SerialGps.setRxBufferSize(512);
  SerialGps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  // Wait until Serial is available, max 1 Second
  while (!Serial && (millis() <= 1000));

  SPI.begin( SCLK_PIN, MISO_PIN, MOSI_PIN );
  SPI.setFrequency(25000000);
  // uncomment for normal SPI mode; default true used for "special" SPI circuit found e.g. on 3.5" RPI HVGA display
  tft.setSpiKludge(true); // false to disable rpi_spi16_mode
  tft.init();
  tft.setRotation(myRotation);
  oTime = millis();
  tft.fillScreen(BLACK);
  Serial.print("Clear Screen = "); Serial.print(millis() - oTime); Serial.println(" ms");
  
  tft.fillRect(0, 0, 480, 30, BLUE);
  tft.setTextSize(2);
  tft.setCursor(10, 8);
  tft.print("AFU - TOOLBOX - V 1.0.0 - DB1IAT");
  tft.setTextSize(2);
  tft.setTextColor(GREEN, BLACK);
  tft.setCursor(0, 40);
  tft.println("AFU-Tool Startup Sequence");
  tft.println("TFT Initialize done");
  Serial.println("TFT-Config DONE ");
  //tft.setFont(&FreeMono9pt7b);
  SerialGps.setRxBufferSize(512);
  SerialGps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  tft.println("Hardware Serial 1 GPS done");
  // Wait until Serial is available, max 1 Second
  // Start Touch
  ts.begin();
  ts.setRotation(myRotation);
  tft.println("Touch Initialize done");
  // Set PinMode Rotary Encoder
  pinMode(rotAPin, INPUT_PULLUP);
  pinMode(rotBPin, INPUT_PULLUP);
  pinMode(pushPin, INPUT_PULLUP);
  // Attach Interrupts for Rotary-Encoder
  attachInterrupt(digitalPinToInterrupt(rotAPin), ISRrotAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotBPin), ISRrotBChange, CHANGE);
  tft.println("Rotary Initialize done");
  Serial.println("Rotary-Config DONE ");
  
  // Check Pushpin to do something if neccesary
  if (digitalRead(pushPin) == LOW) {
  }
  
  // GPS Initialization
  // Reset GPS to default Config
  tft.println("Set GPS default Configuration");
  Serial.println("GPS-UBLOX_M7_Set_Default_Config ");
  SendMemToGps(UBLOX_M7_Set_Default_Config, sizeof(UBLOX_M7_Set_Default_Config), &SerialGps);   
  // Wait to reinitialize GPS
  delay(1000);
  Serial.println("GPS-Config MyInit ");
  // Send the Initalize Sequence to the GPS
  tft.println("Set GPS to UBX Mode");
  SendMemToGps(UBLOX_INIT, sizeof(UBLOX_INIT), &SerialGps);   
  tft.println("GPS Initialization done");
  attachInterrupt(digitalPinToInterrupt(ppsPin), ISRpps, FALLING);
  Serial.println("GPS-Config DONE ");
  delay(100);
  
  // P L L
  tft.println("PLL Initialization...");
  InitPll();
  si5351.set_freq(frequencyA*SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.set_freq(frequencyB*SI5351_FREQ_MULT, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.set_freq(frequencyC*SI5351_FREQ_MULT, SI5351_CLK2);
  si5351.output_enable(SI5351_CLK2, 0);
  si5351.set_phase(SI5351_CLK0, PhaseA);
  si5351.set_phase(SI5351_CLK1, PhaseB);
  si5351.set_phase(SI5351_CLK0, PhaseA);
  si5351.set_phase(SI5351_CLK2, PhaseC);
  si5351.pll_reset(SI5351_PLLA);        
  Serial.println("PLL-Config DONE ");
  tft.println("PLL Initialization done");
  
  // Timer IRQ
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  // 10 times a Second
  timerAlarmWrite(timer, 100000, true);  
  timerAlarmEnable(timer);
  Serial.println("Timer-Config DONE ");
  tft.println("Timer Initialization done");
  tft.println("Button Number 35 RST Restarting ESP32");
  tft.println("Touch anywhere to Start...");
  while (!ts.touched()) {}
  // Display first Screen
  //MapTouchScreen();
  
  // Set the TFT in Button
  ButtonBegin(&tft);
  InitMenu1();
}

//**********************************************************************************
//  L o o p
//**********************************************************************************
void loop(void) {
  if (millis() >= mTime +10000) {
    mTime = millis();
    int Bat = analogRead(A17);
    float BatV = (3294.0 / 4095) * Bat;
    BatV = BatV * 2.680;
    Serial.println(BatV/1000.0);
    //Serial.print("ppsDuration = "); Serial.print(ppsDuration); Serial.println(" ms");
  }
  
  //DoGPS();          GPS is done 10 Times a Second in the Timer IQR
  DoButtons();
  DoRotaryPush();

  if (DoRotaryChanged() == true) {
    //UpdateDisplay();
    if (DisplayMode == 0) {
      DrawPllCanvas();
    }
  }

  // Got a complete Message from the GPS-Module
  if (msgType == MT_NAV_PVT) {
    msgType = 0;
    DrawGpsCanvas();
    //if (DisplayMode == 1) {
    //  DrawGpsCanvas();
    //}
  }

  // Timer
  if (interruptCounter > 0) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    totalInterruptCounter++;
    //Serial.print("An interrupt occurred. Total Number: ");
    //Serial.println(totalInterruptCounter);
  }  
}



