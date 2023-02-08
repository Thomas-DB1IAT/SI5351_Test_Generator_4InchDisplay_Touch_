
//*******************************************************************************
// Rotary encoder push button pin, active low
//*******************************************************************************
static const int pushPin = 32;
// Rotary encoder phase A pin
static const int rotAPin = 25;
// Rotary encoder phase B pin
static const int rotBPin = 26;

void ISRrotAChange();
void ISRrotBChange();
int GetRotAcc();
