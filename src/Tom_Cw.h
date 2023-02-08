
#include <Arduino.h>
#include <si5351.h>

extern int CheckTouchedButton(void); 


void cw_string_proc(String str);
void cw_char_proc(char m);
void cw(bool state);
void ti();
void ta();
void char_space();                             
void word_space();                             
void DoCw(String cw_message, uint8_t ApinBeep, Si5351* Asi5351);
void cw_stopIt(void);
