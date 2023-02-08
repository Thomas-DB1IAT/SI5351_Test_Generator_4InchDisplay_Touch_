#include "Tom_Rotary.h"
#include <Arduino.h>

//*******************************************************************************
// Rotary encoder variables, used by interrupt routines
//*******************************************************************************
volatile int rotState = 0;
volatile int rotAval = 1;
volatile int rotBval = 1;
volatile int rotAcc = 0;

//******************************************************************
// ROTARY Encoder Functions
//******************************************************************

//******************************************************************
// Update rotary encoder accumulator and State Machine. 
// This function is called by the interrupt routines.
//******************************************************************
void UpdateRot() 
{
  // Increment rotAcc if going CW, decrement it if going CCW.
  // Do not increment anything if it was just a glitch.
  switch(rotState) {
    case 0: // Idle state, look for direction
      if(!rotBval) {
        rotState = 1;  // CW 1
      }
      if(!rotAval) {
        rotState = 11; // CCW 1
      }
      break;
    case 1: // CW, wait for A low while B is low
      if(!rotBval) {
        if(!rotAval) {
          rotAcc++;
          rotState = 2; // CW 2
        }
      } else {
        if(rotAval) {
          // It was just a glitch on B, go back to start
          rotState = 0;
        }
      }
      break;
    case 2: // CW, wait for B high
      if(rotBval) {
        rotState = 3; // CW 3
      }
      break;
    case 3: // CW, wait for A high
      if(rotAval) {
        rotState = 0; // back to idle (detent) state
      }
      break;

    case 11: // CCW, wait for B low while A is low
      if(!rotAval) {
        if(!rotBval) {
          rotAcc--;
          rotState = 12; // CCW 2
        }
      } else {
        if(rotBval) {
          // It was just a glitch on A, go back to start
          rotState = 0;
        }
      }
      break;
    case 12: // CCW, wait for A high
      if(rotAval) {
        rotState = 13; // CCW 3
      }
      break;
    case 13: // CCW, wait for B high
      if(rotBval) {
        rotState = 0; // back to idle (detent) state
      }
      break;
  }
}

//******************************************************************
// Return the current value of the rotaryEncoder 
// counter in an interrupt safe way.
//******************************************************************
int GetRotAcc() 
{
  int rot;
   
  cli();
  rot = rotAcc;
  sei();
  return rot;
}
 
//******************************************************************
// Interrupt routines PIN A
//******************************************************************
void ISRrotAChange()
{
  if(digitalRead(rotAPin)) {
    rotAval = 1;
    UpdateRot();
  } else {
    rotAval = 0;
    UpdateRot();
  }
}
 
//******************************************************************
// Interrupt routines PIN B
//******************************************************************
void ISRrotBChange() 
{
  if(digitalRead(rotBPin)) {
    rotBval = 1;
    UpdateRot();
  } else {
    rotBval = 0;
    UpdateRot();
  }
}
 
