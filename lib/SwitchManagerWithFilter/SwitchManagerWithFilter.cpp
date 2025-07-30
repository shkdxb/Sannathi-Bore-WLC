//                c l a s s   S w i t c h M a n a g e r W i t h F i l t e r
//********************************************^************************************************
/*
  Class for making "switch" objects
  Noise filter Added REV.  1.01  LarryD
 
  inspired by Nick Gammon's "class" below:
  http://gammon.com.au/Arduino/SwitchManager.zip
 
  Example:
  //if used from an external library
  #include <SwitchManagerWithFilter.h>
 
  //"switch" object instantiations
  SwitchManagerWithFilter mySwitch;
 
  void setup()
  {
      //create a "switch" object
      mySwitch.begin (2, handleSwitches);
  }
 
  void loop()
  {
      mySwitch.check();  //check for "switch" operation, do this at loop() speed
  }
 
  //When a valid "switch" operation is detected:
  //- newState will be LOW or HIGH (the state the "switch" is now in)
  //- interval will be how many mS between the opposite state and this state
  //- whichPin will be the pin that caused this change
  //you can share the function amongst multiple switches
  void handleSwitches(const byte newState, const unsigned long interval, const byte whichPin)
   {
      //do something
   }
*/
 
#include <Arduino.h>
 
class SwitchManagerWithFilter
{
    enum {debounceTime = 10, noSwitch = -1};
 
    typedef void (*handlerFunction)
    (
      const byte newState,
      const unsigned long interval,
      const byte whichSwitch
    );
 
    int pinNumber_;
    handlerFunction f_;
    byte oldSwitchState_;
    unsigned long switchPressTime_;  //when the "switch" last changed state
    unsigned long lastLowTime_;
    unsigned long lastHighTime_;
    byte sampleCounter_;
 
  public:
    //constructor
    SwitchManagerWithFilter()
    {
      pinNumber_       = noSwitch;
      f_               = NULL;
      oldSwitchState_  = 0;
      switchPressTime_ = 0;
      lastLowTime_     = 0;
      lastHighTime_    = 0;
      sampleCounter_   = 0;
    }
 
    //****************************************
    void begin (const int pinNumber, handlerFunction f)
    {
      pinNumber_ = pinNumber;
      f_ = f;
 
      //*********************
      if (pinNumber_ != noSwitch)
      {
        pinMode (pinNumber_, INPUT_PULLUP);
 
        oldSwitchState_  = digitalRead(pinNumber_);
      }
    }  //END of    begin()
 
    //****************************************
    void check()
    {
      //*********************
      //we need a valid pin number and a valid function to call
      if (pinNumber_ == noSwitch || f_ == NULL)
      {
        return;
      }
 
      //*********************
      //time to check the "switch"?
      if (millis () - switchPressTime_ < debounceTime)
      {
        //it's not time yet
        return;
      }
 
      //time is up, get ready for next sequence
      switchPressTime_ = millis();
 
      byte switchState = digitalRead(pinNumber_);
 
      //*********************
      //has the "switch" changed state?
      if (switchState == oldSwitchState_)
      {
       // reset the counter as no state changed was detected
        sampleCounter_ = 0;
 
        return;
      }
 
      //the "switch" has changed state
      //this is used to filter electrical circuit noise
      sampleCounter_++;
 
      //Must read the "switch" sequential 2 times before we validate a "switch" change.
      //i.e. if debounceTime is 10ms, it will take 20ms to validate a "switch" change.
      //With an input sample rate of 10ms, input signals > 20ms are guaranteed to be captured.
      //Signals 10-20ms might be captured, with signals < 10ms guaranteed not to be captured.
 
      //*********************
      //is this the 2nd time in two sequential reads?
      if (sampleCounter_ >= 2)
      {
        //remember for next time
        oldSwitchState_ =  switchState;
 
        //get ready for the next 2 samples
        sampleCounter_ = 0;
 
        //*********************
        //is the "switch" LOW or HIGH
        if (switchState == LOW)
        {
          lastLowTime_ = switchPressTime_;
          f_ (LOW, lastLowTime_ -  lastHighTime_, pinNumber_);
        }
 
        //*********************
        //"switch" must be HIGH
        else
        {
          lastHighTime_ = switchPressTime_;
          f_ (HIGH, lastHighTime_ - lastLowTime_, pinNumber_);
        }
      }
    }  //END of   check()
 
};  //END of   class SwitchManagerWithFilter