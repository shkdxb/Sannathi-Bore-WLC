#include <TinyWireM.h>
#include <SendOnlySoftwareSerial.h>
#include <EEPROM.h>

#define TX_PIN 1       // PB1
#define POT_ADC_PIN A3 // PB3
#define ACS712_PIN A2  // PB4

#define PCF_ADDR 0x20

#define RELAY_BIT 0b00000001      // P0
#define FLOW_BIT 0b00001000       // P3 (as per your schematic)
#define RANGE_POT_BIT 0b00000100  // P2
#define CENTER_POT_BIT 0b00000010 // P1
#define MODE_P4_BIT 0b00010000    // P4
#define MODE_P5_BIT 0b00100000    // P5
#define ERROR_LED_BIT 0b01000000  // P6

SendOnlySoftwareSerial mySerial(TX_PIN);

uint8_t pcfState = 0x00;
uint16_t internalRefMV = 1100;
bool relayIsOn = false;
unsigned long lastRelayOnTime = 0;
unsigned long lastFlowPulseTime = 0;
bool lastFlowLevel = true;
bool flowActive = false;

bool writeToPCF(uint8_t bitMask, bool setBit)
{
  if (setBit)
    pcfState |= bitMask;
  else
    pcfState &= ~bitMask;
  TinyWireM.beginTransmission(PCF_ADDR);
  TinyWireM.send(pcfState);
  return (TinyWireM.endTransmission() == 0);
}

bool readPCF(uint8_t &value)
{
  TinyWireM.requestFrom(PCF_ADDR, 1);
  if (TinyWireM.available())
  {
    value = TinyWireM.receive();
    return true;
  }
  return false;
}

void saveInternalRefToEEPROM(uint16_t mv)
{
  EEPROM.update(0, lowByte(mv));
  EEPROM.update(1, highByte(mv));
}

void loadInternalRefFromEEPROM()
{
  uint8_t lo = EEPROM.read(0);
  uint8_t hi = EEPROM.read(1);
  internalRefMV = (hi << 8) | lo;
  if (internalRefMV < 900 || internalRefMV > 1200)
    internalRefMV = 1100;
}

long readVcc()
{
  ADMUX = (1 << MUX3) | (1 << MUX2) | (1 << MUX1);
  delay(2);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  return ((long)internalRefMV * 1023L) / ADC;
}

void blinkError(uint8_t count)
{
  for (uint8_t i = 0; i < count; i++)
  {
    writeToPCF(ERROR_LED_BIT, true);
    delay(200);
    writeToPCF(ERROR_LED_BIT, false);
    delay(200);
  }
  delay(1000);
}

void updateFlowStatus()
{
  uint8_t val;
  if (!readPCF(val))
    return;
  bool flowPin = val & FLOW_BIT;
  if (lastFlowLevel && !flowPin)
    lastFlowPulseTime = millis();
  lastFlowLevel = flowPin;
  flowActive = (millis() - lastFlowPulseTime < 2000);
}

void setRelay(bool on)
{
  if (!writeToPCF(RELAY_BIT, on))
  {
    mySerial.println(F("ERROR: Relay write failed."));
    blinkError(4);
  }
  relayIsOn = on;
  if (on)
    lastRelayOnTime = millis();
}

int readPot(bool isCenter)
{
  writeToPCF(RANGE_POT_BIT | CENTER_POT_BIT, false);
  writeToPCF(isCenter ? CENTER_POT_BIT : RANGE_POT_BIT, true);
  delay(5);
  return analogRead(POT_ADC_PIN);
}

uint8_t getMode()
{
  uint8_t val;
  if (!readPCF(val))
    return 0xFF;
  bool p4 = val & MODE_P4_BIT;
  bool p5 = val & MODE_P5_BIT;
  if (p4 && p5)
    return 0;
  if (p4 && !p5)
    return 1;
  if (!p4 && p5)
    return 2;
  return 3;
}

void calibrateADC()
{
  analogReference(INTERNAL);
  delay(100);
  int adc = analogRead(POT_ADC_PIN);
  uint16_t ref = (uint32_t)1000 * 1023 / adc;
  saveInternalRefToEEPROM(ref);
  mySerial.print(F("Calibrated Vref = "));
  mySerial.print(ref);
  mySerial.println(F(" mV saved."));
  while (1)
    ;
}

void calibrateCenter()
{
  int adc = analogRead(POT_ADC_PIN);
  mySerial.print(F("Center POT ADC = "));
  mySerial.println(adc);
  while (1)
    ;
}

void calibrateRange()
{
  int adc = analogRead(POT_ADC_PIN);
  mySerial.print(F("Range POT ADC = "));
  mySerial.println(adc);
  while (1)
    ;
}

void runController()
{
  loadInternalRefFromEEPROM();

  while (1)
  {
    updateFlowStatus();
    int centerADC = readPot(true);
    int rangeADC = readPot(false);
    long vccMV = readVcc();

    // Integer scale factor for converting ADC to mA
    int adcToMAx1000 = (vccMV * 1000L) / (1023L * 66L);

    int centerMA = (centerADC * adcToMAx1000) / 1000;
    int rangeMA = (rangeADC * adcToMAx1000) / 2000;
    int currentADC = analogRead(ACS712_PIN);
    int currentMA = (currentADC * adcToMAx1000) / 1000;

    int lowTh = centerMA - rangeMA;
    int highTh = centerMA + rangeMA;

    bool inrushOK = relayIsOn && (millis() - lastRelayOnTime < 5000);
    bool currentOK = (currentMA > lowTh && currentMA < highTh) || inrushOK;
    bool flow = flowActive = (millis() - lastFlowPulseTime < 2000);
    bool shouldRun = flow && currentOK;

    if (!flow)
      blinkError(1);
    else if (!inrushOK && currentMA < lowTh)
      blinkError(2);
    else if (!inrushOK && currentMA > highTh)
      blinkError(3);

    setRelay(shouldRun);

    mySerial.print(F("Flow:"));
    mySerial.print(flow ? F("Y") : F("N"));
    mySerial.print(F(" | I="));
    mySerial.print(currentMA);
    mySerial.print(F(" mA | C="));
    mySerial.print(centerMA);
    mySerial.print(F(" | R="));
    mySerial.print(rangeMA);
    mySerial.print(F(" | Vcc="));
    mySerial.print(vccMV);
    mySerial.print(F("mV | Relay="));
    mySerial.println(shouldRun ? F("ON") : F("OFF"));

    delay(1000);
  }
}

void setup()
{
  TinyWireM.begin();
  mySerial.begin(9600);
  delay(100);

  writeToPCF(ERROR_LED_BIT, true);
  delay(10);
  uint8_t state;
  if (!readPCF(state))
  {
    mySerial.println(F("ERROR: PCF8574 not responding!"));
    while (1)
      ;
  }
  if (!(state & ERROR_LED_BIT))
  {
    mySerial.println(F("ERROR: PCF8574 P6 did not go HIGH!"));
    while (1)
      ;
  }

  uint8_t mode = getMode();
  if (mode == 1)
    calibrateADC();
  else if (mode == 2)
    calibrateCenter();
  else if (mode == 3)
    calibrateRange();
  else
    runController();
}

void loop() {}
