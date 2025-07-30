// Class for managing switch presses
// Author: Nick Gammon
// Date: 18 December 2013


/*
 Example:
 
 #include <SwitchManager.h>
 
 SwitchManager mySwitch;  // declare
 
 // newState will be LOW or HIGH (the is the state the switch is now in)
 // interval will be how many mS between the opposite state and this one
 
 void handleSwitchPress (const byte newState, const unsigned long interval)
   {
   
   }
 
 void setup ()
   {
   mySwitch.begin (2, handleSwitchPress);
   }
 
 void loop ()
   {
   mySwitch.check ();  // check for presses
   }
 
 
 */

#include <Arduino.h>


class SwitchManager
  {
  enum { debounceTime = 10, noSwitch = -1 };
  typedef void (*handlerFunction) (const byte newState, const unsigned long interval);
  
  int pinNumber_;
  handlerFunction f_;
  byte oldSwitchState_;
  unsigned long switchPressTime_;  // when the switch last changed state
  unsigned long lastLowTime_;
  unsigned long lastHighTime_;
  
  public:
  
     // constructor
     SwitchManager () 
       {
       pinNumber_ = noSwitch;
       f_ = NULL;
       oldSwitchState_  = HIGH;
       switchPressTime_ = 0;
       lastLowTime_  = 0;
       lastHighTime_ = 0;
       }
       
     void begin (const int pinNumber, handlerFunction f)
       {
       pinNumber_ = pinNumber;
       f_ = f;
       if (pinNumber_ != noSwitch)
         pinMode (pinNumber_, INPUT_PULLUP);
       }  // end of begin
       
     void check ()
       {
       // we need a valid pin number and a valid function to call
       if (pinNumber_ == noSwitch || f_ == NULL)
         return;
        
        // see if switch is open or closed
        byte switchState = digitalRead (pinNumber_);
        
        // has it changed since last time?
        if (switchState != oldSwitchState_)
          {
          // debounce
          if (millis () - switchPressTime_ >= debounceTime)
             {
             switchPressTime_ = millis ();  // when we closed the switch 
             oldSwitchState_ =  switchState;  // remember for next time 
             if (switchState == LOW)
               {
               lastLowTime_ = switchPressTime_;
               f_ (LOW, lastLowTime_ -  lastHighTime_);
               }
             else
               {
               lastHighTime_ = switchPressTime_;
               f_ (HIGH, lastHighTime_ - lastLowTime_);
               }
             
             }  // end if debounce time up
          }  // end of state change
       }  // end of operator ()
       
  };  // class SwitchManager
