// The number of symbols in the WSPR transmission
#define WSPR_SYMBOLS_LENGTH       162       
// The number of symbols in the WSPR transmission
#define WSPR_SYMBOLS              162       
// 1.46484375 bauds
#define WSPR_BAUD_RATE            (375.0 / 256.0)                 
// 256 / 375 = 682.67ms
#define WSPR_SYMBOL_DURATION      (1000.0 / WSPR_BAUD_RATE)      

// ~1.46 Hz
#define WSPR_TONE_SPACING       146            
// CTC value for WSPR
#define WSPR_CTC                10672          
// 14095600 Dial (+1400 Hz to +1600 Hz -> 14097000 – 14097200)
#define WSPR_DEFAULT_FREQ       1409560000ULL  
// 0..161
#define WSPR_SYMBOL_COUNT       162            
#define WSPR_BIT_COUNT          162


// ~1.46 Hz
#define WSPR_TONE_SPACING       146            
// CTC value for WSPR
#define WSPR_CTC                10672          
// 14095600 Dial (+1400 Hz to +1600 Hz -> 14097000 – 14097200)
#define WSPR_DEFAULT_FREQ       1409560000ULL  
// 0..161
#define WSPR_SYMBOL_COUNT       162            
#define WSPR_BIT_COUNT          162
