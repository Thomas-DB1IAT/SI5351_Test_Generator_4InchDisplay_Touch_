#include <Arduino.h>
#include "Tom_WSPR.h"
#include <si5351.h>

#define DebugMode false

#define pushPin 2

Si5351* Wspr_si5351 = NULL;

//*******************************************************************************
// WSPR Symbol Memory
//*******************************************************************************
uint8_t mySymbols[WSPR_SYMBOL_COUNT] =
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


//*******************************************************************************
// WSPR Dail Frequency (Hertz * 100)
//*******************************************************************************
uint64_t WsprBandFreq[14]    = {13600000, 47420000, 183660000, 359260000, 
                                528720000, 703860000, 1013870000, 1409560000, 
                                1810460000, 2109460000, 2492460000, 2812460000, 
                                5029300000, 70091000000};

char     WsprBandTxt[14][6]  = {"LF136", "MF472", "160m ", "80m  ",
                                "60m  ", "40m  ", "30m  ", "20m  ",
                                "17m  ", "15m  ", "12m  ", "10m  ",
                                "6m   ", "4m   " };

int8_t   WsprBandUsed        = 7;   // default 20m 
uint8_t  WsprBandMax         = 14;
//*******************************************************************************

//*******************************************************************************
// WSPR TX Percent
// TX can start at even Minutes 0, 2, 4, 6, 8 
// and takes slightly less then 2 Minutes (aprox. 112 Seconds)
//*******************************************************************************
int8_t        WsprTxPercent    =    0;   // Actual used Percentage
int8_t        WsprTxPerStep    =   10;   // Step width to change the Percentage
int8_t        WsprTxPerSlot    =   -1;   // Use ever Slot to TX -> -1 = 0%
uint8_t       WsprSlotCounter  =    0;   // Tx Slot Counter

//*******************************************************************************
// WSPR TX Delay at Startup default 1000 mS
uint16_t WsprTxDelay = 1000; //

//*******************************************************************************
// WSPR NF BASE   1500 Hz 
uint16_t WsprNfBase = 1500;

//*******************************************************************************
// WSPR NF Offset +-100 Hz
int16_t WsprNfOffset = 0;

//*******************************************************************************
struct WsprData {
  char   Callsign[7]   = "DB1IAT";     // Callsign used for WSPR
  char   Locator[7]    = "AA00Aa";     // Locator  used for WSPR    (default Aa00Aa)
  byte   Power         =  20;          // Powerlevel used for WSPR  (default 100 mW)
};

//*******************************************************************************
// Will be used in TX Mode
WsprData TxData;
// Will be used in Menu Mode to Edit
WsprData MenuData;


char   MyValidator[10] = "DEADBEEF";   // Validator for EEPROM
char   MyCallsign[7]   = "DB1IAT";     // Callsign used for WSPR
char   MyLocator[7]    = "AA00Aa";     // Locator  used for WSPR    (default Aa00Aa)
byte   MyPower         =  20;          // Powerlevel used for WSPR  (default 100 mW)
char   GpsLocator[7]   = "AA00Aa";     // Locator reported form GPS 

int    GpsSyncCount = 0;  
bool   GpsSyncOk    = false;  // GPS Synchroniszied 
bool   RtcSyncOk    = false;  // RTC is Syncronized 
bool   WsprTx       = false;  // Allready TX ing
bool   DoTx         = false;  // Want to go to TX

bool   FreqTestMode = false;
int    FreqTestTime = 0; 

int    WsprPwrMode = 0;
int    WsprItem    = 0;

//******************************************************************
// Set MyPower matching to WsprPwrMode
//******************************************************************
void SetPwrMode(uint8_t MyValue) {
  switch (WsprPwrMode) {
    case 0: {MyPower =  20; break; }   // Powerlevel used for WSPR  (  100 mW)
    case 1: {MyPower =  23; break; }   // Powerlevel used for WSPR  (  200 mW)
    case 2: {MyPower =  27; break; }   // Powerlevel used for WSPR  (  500 mW)
    case 3: {MyPower =  30; break; }   // Powerlevel used for WSPR  ( 1000 mW)
    
    default: {MyPower = 20; break; }
   }
}   

//******************************************************************
// Set WsprTxPerSlot matching to WsprTxPercent
//******************************************************************
void SetTxSlot(uint8_t MyValue) {
  switch (MyValue) {
    case 100: {WsprTxPerSlot = 1;   break; }
    case  90: {WsprTxPerSlot = 2;   break; }
    case  80: {WsprTxPerSlot = 3;   break; }
    case  70: {WsprTxPerSlot = 4;   break; }
    case  60: {WsprTxPerSlot = 5;   break; }
    case  50: {WsprTxPerSlot = 6;   break; }
    case  40: {WsprTxPerSlot = 7;   break; }
    case  30: {WsprTxPerSlot = 8;   break; }
    case  20: {WsprTxPerSlot = 9;   break; }
    case  10: {WsprTxPerSlot = 10;  break; }
    case   0: {WsprTxPerSlot = -1;  break; }  // TX never

    default:  {WsprTxPerSlot = -1;  break; } 
  }
  // Reset the SlotCounter
  WsprSlotCounter = 0;
}

//********************************************************************
// TX WSPR Items
//
//  11200 Offset matches 1400 Hz NF Tested at 20m
//  21200 Offset matches 1500 Hz NF Tested at 20m
//  31200 Offset matches 1600 Hz NF Tested at 20m
//
//********************************************************************
void TxWsprItems() {
  WsprTx = true;
    
  // Set the desired PowerMode
  switch (WsprPwrMode) {
    case 0: {Wspr_si5351->drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); break;}
    case 1: {Wspr_si5351->drive_strength(SI5351_CLK1, SI5351_DRIVE_4MA); break;}
    case 2: {Wspr_si5351->drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA); break;}
    case 3: {Wspr_si5351->drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA); break;}
    default: 
            {Wspr_si5351->drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); break;}
  }

  unsigned long TxF = WsprBandFreq[WsprBandUsed]+(WsprNfBase*100)+(WsprNfOffset*100);
  //SerPrintTime();
  if (DebugMode) Serial.println("Start WSPR TX"); 
  delay(WsprTxDelay);  
  //SerPrintTime();
  if (DebugMode) Serial.println("Enable CLK1");
  Wspr_si5351->output_enable(SI5351_CLK1, 1);
  if (DebugMode) {Serial.println("WSPR TX LOOP"); }
  if (DebugMode) {Serial.print("WsprNfOffset: "); Serial.println(String(uint32_t(WsprNfOffset))); }
  if (DebugMode) {Serial.print("TX: ");           Serial.println(String(int32_t(TxF))); }

  for (int i=0; i<162; i++) {
    WsprItem = i;

    unsigned long TxFreq = WsprBandFreq[WsprBandUsed]+(WsprNfBase*100)+(WsprNfOffset*100)+(mySymbols[i] * WSPR_TONE_SPACING);
    Wspr_si5351->set_freq(TxFreq, SI5351_CLK1);
    unsigned long om = millis();

    //MainScreenDisplay();
    
    /*
    // WSPR Item
    display.setCursor(0, 5*8);
    display.print("ITEM ");
    display.setCursor(32, 5*8);
    display.print(WsprItem);
    // WSPR Tone
    display.setCursor(11*7, 5*8);
    display.print("TONE ");
    display.setCursor(108, 5*8);
    display.print(mySymbols[WsprItem]);
    display.display();
    */
    om = millis() -om;
    delay(685-om);
    // Check Pushpin to Interrupt the Transmission
    if (digitalRead(pushPin) == LOW) { 
      delay(50);
      while (digitalRead(pushPin) != HIGH) {
        delay(10);
      }
      //SerPrintTime();
      if (DebugMode) Serial.println("WSPR TX CANCELED BY USER"); 
      break;
    }
  }
  //SerPrintTime();
  if (DebugMode) Serial.println("Finish WSPR TX"); 
  // Disable the Transmitter
  if (DebugMode) Serial.println("Disable CLK1");
  Wspr_si5351->output_enable(SI5351_CLK1, 0);
  WsprItem = 0;
  WsprTx = false;
  DoTx = false;
}


void WsprItemsClear() {
  for (int i=0; i<WSPR_SYMBOL_COUNT; ++i) {
    mySymbols[i] = 0;
  }
}

//******************************************************************
//
//******************************************************************
void wspr_message_prep(char * call, char * loc, uint8_t dbm)
{
  Serial.println("wspr_message_prep");
  // Callsign validation and padding
  // -------------------------------

  // If only the 2nd character is a digit, then pad with a space.
  // If this happens, then the callsign will be truncated if it is
  // longer than 5 characters.
  if((call[1] >= '0' && call[1] <= '9') && (call[2] < '0' || call[2] > '9'))
  {
    memmove(call + 1, call, 5);
    call[0] = ' ';
  }

  // Now the 3rd charcter in the callsign must be a digit
  if(call[2] < '0' || call[2] > '9')
  {
    // TODO: need a better way to handle this
    call[2] = '0';
  }

  // Ensure that the only allowed characters are digits and
  // uppercase letters
  uint8_t i;
  for(i = 0; i < 6; i++)
  {
    call[i] = toupper(call[i]);
    if(!(isdigit(call[i]) || isupper(call[i])))
    {
      call[i] = ' ';
    }
  }

  memcpy(MyCallsign, call, 6);

  bool invalid = false;
  // Grid locator validation
  for(i = 0; i < 4; i++)
  {
    loc[i] = toupper(loc[i]);
    if(!(isdigit(loc[i]) || (loc[i] >= 'A' && loc[i] <= 'R')))
    {
      invalid = true;
      break;
    }
  }
  if(invalid)
  {
    memcpy(MyLocator, "AA00", 4);
  }
  else
  {
    memcpy(MyLocator, loc, 4);
  }  

  memcpy(MyLocator, loc, 4);

  // Power level validation
  // Only certain increments are allowed
  if(dbm > 60)
  {
    dbm = 60;
  }
  const uint8_t valid_dbm[19] =
    {0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40,
     43, 47, 50, 53, 57, 60};
  for(i = 0; i < 19; i++)
  {
    if(dbm == valid_dbm[i])
    {
      MyPower = dbm;
    }
  }
  // If we got this far, we have an invalid power level, so we'll round down
  for(i = 1; i < 19; i++)
  {
    if(dbm < valid_dbm[i] && dbm >= valid_dbm[i - 1])
    {
      MyPower = valid_dbm[i - 1];
    }
  }
}


//******************************************************************
//
//******************************************************************
void convolve(uint8_t * c, uint8_t * s, uint8_t message_size, uint8_t bit_size)
{
  Serial.println("wspr_convolve");
  uint32_t reg_0 = 0;
  uint32_t reg_1 = 0;
  uint32_t reg_temp = 0;
  uint8_t input_bit, parity_bit;
  uint8_t bit_count = 0;
  uint8_t i, j, k;

  for(i = 0; i < message_size; i++)
  {
    for(j = 0; j < 8; j++)
    {
      // Set input bit according the MSB of current element
      input_bit = (((c[i] << j) & 0x80) == 0x80) ? 1 : 0;

      // Shift both registers and put in the new input bit
      reg_0 = reg_0 << 1;
      reg_1 = reg_1 << 1;
      reg_0 |= (uint32_t)input_bit;
      reg_1 |= (uint32_t)input_bit;

      // AND Register 0 with feedback taps, calculate parity
      reg_temp = reg_0 & 0xf2d05351;
      parity_bit = 0;
      for(k = 0; k < 32; k++)
      {
        parity_bit = parity_bit ^ (reg_temp & 0x01);
        reg_temp = reg_temp >> 1;
      }
      s[bit_count] = parity_bit;
      bit_count++;

      // AND Register 1 with feedback taps, calculate parity
      reg_temp = reg_1 & 0xe4613c47;
      parity_bit = 0;
      for(k = 0; k < 32; k++)
      {
        parity_bit = parity_bit ^ (reg_temp & 0x01);
        reg_temp = reg_temp >> 1;
      }
      s[bit_count] = parity_bit;
      bit_count++;
      if(bit_count >= bit_size)
      {
        break;
      }
    }
  }
}

uint8_t wspr_code(char c)
{
  // Validate the input then return the proper integer code.
  // Return 255 as an error code if the char is not allowed.
  if(isdigit(c))
  {
    return (uint8_t)(c - 48);
  }
  else if(c == ' ')
  {
    return 36;
  }
  else if(c >= 'A' && c <= 'Z')
  {
    return (uint8_t)(c - 55);
  }
  else
  {
    return 255;
  }
}

//******************************************************************
//
//******************************************************************
void wspr_bit_packing(uint8_t * c)
{
  Serial.println("wspr_bit_packing");
  uint32_t n, m;

  n = wspr_code(MyCallsign[0]);
  n = n * 36 + wspr_code(MyCallsign[1]);
  n = n * 10 + wspr_code(MyCallsign[2]);
  n = n * 27 + (wspr_code(MyCallsign[3]) - 10);
  n = n * 27 + (wspr_code(MyCallsign[4]) - 10);
  n = n * 27 + (wspr_code(MyCallsign[5]) - 10);

  m = ((179 - 10 * (MyLocator[0] - 'A') - (MyLocator[2] - '0')) * 180) +
    (10 * (MyLocator[1] - 'A')) + (MyLocator[3] - '0');
  m = (m * 128) + MyPower + 64;

  // Callsign is 28 bits, locator/power is 22 bits.
  // A little less work to start with the least-significant bits
  c[3] = (uint8_t)((n & 0x0f) << 4);
  n = n >> 4;
  c[2] = (uint8_t)(n & 0xff);
  n = n >> 8;
  c[1] = (uint8_t)(n & 0xff);
  n = n >> 8;
  c[0] = (uint8_t)(n & 0xff);

  c[6] = (uint8_t)((m & 0x03) << 6);
  m = m >> 2;
  c[5] = (uint8_t)(m & 0xff);
  m = m >> 8;
  c[4] = (uint8_t)(m & 0xff);
  m = m >> 8;
  c[3] |= (uint8_t)(m & 0x0f);
  c[7] = 0;
  c[8] = 0;
  c[9] = 0;
  c[10] = 0;
}

//******************************************************************
//
//******************************************************************
void wspr_interleave(uint8_t * s)
{
  Serial.println("wspr_interleave");
  uint8_t d[WSPR_BIT_COUNT];
  uint8_t rev, index_temp, i, j, k;

  i = 0;

  for(j = 0; j < 255; j++)
  {
    // Bit reverse the index
    index_temp = j;
    rev = 0;

    for(k = 0; k < 8; k++)
    {
      if(index_temp & 0x01)
      {
        rev = rev | (1 << (7 - k));
      }
      index_temp = index_temp >> 1;
    }

    if(rev < WSPR_BIT_COUNT)
    {
      d[rev] = s[i];
      i++;
    }

    if(i >= WSPR_BIT_COUNT)
    {
      break;
    }
  }

  memcpy(s, d, WSPR_BIT_COUNT);
}

//******************************************************************
//
//******************************************************************
void wspr_merge_sync_vector(uint8_t * g, uint8_t * symbols)
{
  Serial.println("wspr_merge");
  uint8_t i;
  const uint8_t sync_vector[WSPR_SYMBOL_COUNT] =
  {1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0,
   1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0,
   0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1,
   0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0,
   1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
   0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
   1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
   1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0};

  for(i = 0; i < WSPR_SYMBOL_COUNT; i++)
  {
    symbols[i] = sync_vector[i] + (2 * g[i]);
  }
}


//******************************************************************
//
//******************************************************************
void wspr_encode(char * call, char * loc, uint8_t dbm, uint8_t * symbols)
{
  Serial.println("wspr_encode");
  
  WsprItemsClear();
  
  // Ensure that the message text conforms to standards
  // --------------------------------------------------
  wspr_message_prep(call, loc, dbm);

  // Bit packing
  // -----------
  uint8_t c[11];
  wspr_bit_packing(c);

  // Convolutional Encoding
  // ---------------------
  uint8_t s[WSPR_SYMBOL_COUNT];
  convolve(c, s, 11, WSPR_BIT_COUNT);

  // Interleaving
  // ------------
  wspr_interleave(s);

  // Merge with sync vector
  // ----------------------
  wspr_merge_sync_vector(s, symbols);
}
