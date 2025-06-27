#include <ACS712.h>
#include <SendOnlySoftwareSerial.h>

#define FLOW_PIN 0     // PB0 - INT0
#define CURRENT_PIN A1 // PB2
#define POT_PIN A3     // PB3
#define RELAY_PIN 4    // PB4
#define LED_PIN 1      // PB1

ACS712 ACS(CURRENT_PIN, 5.0);
SendOnlySoftwareSerial SerialOut(FLOW_PIN);

volatile unsigned long lastFlowTime = 0;
bool motorOn = true;

void flowISR()
{
  lastFlowTime = millis();
}

void setup()
{
  pinMode(FLOW_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);
  SerialOut.begin(9600);
  SerialOut.println("Interrupt Mode Active");

  attachInterrupt(0, flowISR, RISING); // INT0 = PB0
  delay(500);
  ACS.autoMidPoint();
}

void loop()
{
  bool flowOK = (millis() - lastFlowTime) < 3000;

  float amps = ACS.mA_AC(100) / 1000.0;

  int potADC = analogRead(POT_PIN);
  float center = map(potADC, 0, 1023, 2, 6);
  float range = map(potADC, 0, 1023, 1, 3);
  float low = center - range;
  float high = center + range;

  bool fault = (amps < low || amps > high || !flowOK);

  if (fault && motorOn)
  {
    digitalWrite(RELAY_PIN, LOW);
    motorOn = false;
    SerialOut.println("Motor OFF - FAULT");
  }
  else if (!fault && !motorOn)
  {
    digitalWrite(RELAY_PIN, HIGH);
    motorOn = true;
    SerialOut.println("Motor ON");
  }

  digitalWrite(LED_PIN, HIGH);
  delay(fault ? 100 : 500);
  digitalWrite(LED_PIN, LOW);
  delay(fault ? 100 : 500);

  SerialOut.print("Amps: ");
  SerialOut.print(amps, 2);
  SerialOut.print(" | Flow: ");
  SerialOut.print(flowOK ? "YES" : "NO");
  SerialOut.print(" | Range: ");
  SerialOut.print(low);
  SerialOut.print("-");
  SerialOut.print(high);
  SerialOut.print(" | Motor: ");
  SerialOut.println(motorOn ? "ON" : "OFF");
}
