#include <Arduino.h>
#include "Tom_Cw.h"
#include <si5351.h>

Si5351* MySi5351 = NULL;

uint16_t duration = 40;    //75
uint16_t hz = 750;         
uint8_t  pinBeep = 2;
uint8_t  stopIt  = 0;

bool     MyTX = false;

void DoCw(String cw_message, uint8_t ApinBeep, Si5351* Asi5351) {
  pinBeep = ApinBeep;
  MySi5351 = Asi5351;
  stopIt = 0;
  // CW-String ausgeben
  Serial.println(cw_message);
  cw_string_proc(cw_message, true);
  if (stopIt == 0) {
    delay(500);                           
    // CW Tone ON
    cw(true);
    delay(3000);                         
    // CW Tone OFF
    cw(false);
    delay(500); 
  }
}

void cw_stopIt(void) {
  stopIt = 1;
}

//=====================================================
// processing string to characters
//=====================================================
void cw_string_proc(String str, bool TX) {
  MyTX = TX;
  for (uint8_t j = 0; j < str.length(); j++) {
    cw_char_proc(str[j]);
    if (stopIt == 1) break;
  }
}

//=====================================================
// processing characters to Morse code symbols
//=====================================================
void cw_char_proc(char m) {
  String s;
  if (m == ' ') {                                      
    word_space(); 
    return;
  }

  // ACSII, case change a-z to A-Z
  if (m > 96)        
  if (m < 123)
  m -= 32;

   // Morse code 
  switch (m) {
  case 'A': s = ".-#";     break;
  case 'B': s = "-...#";   break;
  case 'C': s = "-.-.#";   break;
  case 'D': s = "-..#";    break;
  case 'E': s = ".#";      break;
  case 'F': s = "..-.#";   break;
  case 'G': s = "--.#";    break;
  case 'H': s = "....#";   break;
  case 'I': s = "..#";     break;
  case 'J': s = ".---#";   break;
  case 'K': s = "-.-#";    break;
  case 'L': s = ".-..#";   break;
  case 'M': s = "--#";     break;
  case 'N': s = "-.#";     break;
  case 'O': s = "---#";    break;
  case 'P': s = ".--.#";   break;
  case 'Q': s = "--.-#";   break;
  case 'R': s = ".-.#";    break;
  case 'S': s = "...#";    break;
  case 'T': s = "-#";      break;
  case 'U': s = "..-#";    break;
  case 'V': s = "...-#";   break;
  case 'W': s = ".--#";    break;
  case 'X': s = "-..-#";   break;
  case 'Y': s = "-.--#";   break;
  case 'Z': s = "--..#";   break;

  case '1': s = ".----#";  break;
  case '2': s = "..---#";  break;
  case '3': s = "...--#";  break;
  case '4': s = "....-#";  break;
  case '5': s = ".....#";  break;
  case '6': s = "-....#";  break;
  case '7': s = "--...#";  break;
  case '8': s = "---..#";  break;
  case '9': s = "----.#";  break;
  case '0': s = "-----#";  break;

  case '?': s = "..--..#"; break;
  case '=': s = "-...-#"; break;
  case ',': s = "--..--#"; break;
  case '/': s = "-..-.#";  break;
  }

  for (uint8_t i = 0; i < 7; i++) {
    if (CheckTouchedButton() == 25) {
      stopIt = 1;
      cw(false);
      break;
    }
    switch (s[i]) {
      case '.': ti();  break;                         // TI
      case '-': ta();  break;                         // TA
      case '#': char_space(); return;                 // end of Morse code symbol
    }
  }
}

//=====================================================
// TX DIT
//=====================================================
void ti() {
  // TX TI
  cw(true);                                       
  delay(duration);
  // stop TX TI
  cw(false);                                      
  delay(duration);
}

//=====================================================
// TX DAH
//=====================================================
void ta() {
  // TX TA
  cw(true);                                       
  delay(3 * duration);
  // stop TX TA
  cw(false);                                      
  delay(duration);
}

//=====================================================
// 3x, 
// 1 from element-end + 2 new
//=====================================================
void char_space() {                             
  delay(2 * duration);                          
}

//=====================================================
// 7x, 
// 1 from element-end + 6 new
//=====================================================
void word_space() {                             
  delay(6 * duration);                          
}

//=====================================================
// TX-CW, TX-LED, 750 Hz sound
//=====================================================
void cw(bool state) {        
  if (state) {
    if (MyTX) MySi5351->output_enable(SI5351_CLK0, 1);
    //digitalWrite(PIN_TX, HIGH);
    tone(pinBeep, hz);
  }
  else {
    if (MyTX) MySi5351->output_enable(SI5351_CLK0, 0);
    //digitalWrite(PIN_TX, LOW);
    noTone(pinBeep);
  }
}
