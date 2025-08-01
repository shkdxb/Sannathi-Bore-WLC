#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PZEM004Tv30.h>
#include <JC_Button.h>
// #include <LibPrintf.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// --------------------- Pin Definitions (ESP32) -------------------------
// GPIO6 to GPIO11 → Used for flash memory (SPI), do not use.
// GPIO1 & GPIO3 → Used for Serial; only use if you're not using USB serial.
// GPIO12,    GPIO15 → Need careful use during boot(strapping pins).
// GPIO16 and GPIO17 Serial2
// GPIO21 GPIO22 SDA and SCL
// 5,18, 19, 23 SPI
// Pin 34 ~ 39 INPUT only  - no output or INPUT_PULLUP or INPUT_PULLDOWN
//
#define FLOAT_UGT_PIN 12
#define FLOAT_OHT_PIN 13
#define MOTOR_STATUS_LED 14
#define KEY_SET 26
#define KEY_UP 25
#define KEY_DOWN 27
#define SW_AUTO 32
#define SW_MANUAL 33
#define MOTOR_RELAY_PIN 4
#define ERROR_LED 5

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define I2C_SDA 21
#define I2C_SCL 22

// // --------------------- Globals -------------------------
// Button btnSET(KEY_SET);
// Button btnUP(KEY_UP);
// Button btnDN(KEY_DOWN);
// pin assignments
const byte DN_PIN(KEY_DOWN), // connect a button switch from this pin to ground
    UP_PIN(KEY_UP),          // ditto
    SET_PIN(KEY_SET);

Button btnSET(SET_PIN), btnUP(UP_PIN), btnDN(DN_PIN); // define the buttons

const unsigned long
    REPEAT_FIRST(500), // ms required before repeating on long press
    REPEAT_INCR(100);  // repeat interval for long press
const int
    MIN_COUNT(0),
    MAX_COUNT(59);

bool calibCancelled = 0;

LiquidCrystal_I2C lcd(0x3F, 16, 2); // Address may be 0x3F on some modules

// HardwareSerial PZEMSerial(2);
// PZEM004Tv30 pzem(PZEMSerial);
// PZEM004Tv30 pzem(&Serial2);
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN); // RX, TX

struct Settings
{
    float overVoltage = 250.0;
    float underVoltage = 180.0;
    float overCurrent = 6.5;
    float underCurrent = 0.3;
    float minPF = 0.3;
    unsigned int onTime = 5;
    unsigned int offTime = 15;
    bool dryRun = false;
    bool detectVoltage = false;
    bool detectCurrent = false;
    bool cyclicTimer = false;
} settings;

float voltage = 0, current = 0, power = 0, pf = 0, energy = 0;
char errorMessage[17] = "No ERROR";

bool inMenu = false;
bool motorRunning = false;
bool manulallyON = 0;
bool calibMode = 1;
int systemMode = 0;
int menuIndex = 0;
int error = 0;
unsigned long lastOnTime = 0;
unsigned long lastOffTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long lastPzemRead = 0;
unsigned long lastInteractionTime = 0;
unsigned long lastScreenSwitch = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lasterrorTime = 0;
unsigned long buttonPressStart = 0;
unsigned long lastRepeatTime = 0;
const unsigned long pzemReadInterval = 1000;
const unsigned long repeatInterval = 200;
bool ledState = false;
const uint8_t totalMenuItems = 11;
static uint8_t screenIndex = 0;

// --------------------- Function Declarations -------------------------
void showStatusScreen();
void onUpClick();
void onDownClick();
void onSetClick();
void showMenu();
void calibrateMotor();
void saveSettings();
void loadSettings();
void readPzemValues();
void blinkLED(int pin);
int checkSystemStatus();
void buttonCheck();
void scrollMessage(const char *message, uint8_t row, uint16_t delayMs = 300);
// #include "soc/gpio_struct.h" // For GPIO register access

// void printGpioInputs()
// {
//     uint32_t gpio_low = GPIO.in;        // GPIO0 to GPIO31
//     uint32_t gpio_high = GPIO.in1.data; // GPIO32 to GPIO39

//     Serial.println("GPIO 0-31:");
//     for (int i = 31; i >= 0; i--)
//     {
//         Serial.print((gpio_low >> i) & 1);
//         if (i % 4 == 0)
//             Serial.print(" ");
//     }
//     Serial.println();

//     Serial.println("GPIO 32-39:");
//     for (int i = 39; i >= 32; i--)
//     {
//         Serial.print((gpio_high >> (i - 32)) & 1);
//         if ((i - 32) % 4 == 0)
//             Serial.print(" ");
//     }
//     Serial.println();
// }

void scrollMessage(const char *message, uint8_t row, uint16_t delayMs)
{
    static uint32_t lastUpdate = 0;
    static uint8_t index = 0;
    static const char *prevMessage = nullptr;

    // Reset scrolling if a new message is passed
    if (message != prevMessage)
    {
        index = 0;
        prevMessage = message;
    }

    uint8_t messageLen = strlen(message);

    if (millis() - lastUpdate >= delayMs)
    {
        lastUpdate = millis();

        char displayBuffer[17]; // 16 chars + null terminator

        if (messageLen <= 16)
        {
            // No need to scroll, just center it
            uint8_t pad = (16 - messageLen) / 2;
            memset(displayBuffer, ' ', sizeof(displayBuffer));
            memcpy(displayBuffer + pad, message, messageLen);
        }
        else
        {
            // Scroll if message is longer than 16 characters
            strncpy(displayBuffer, message + index, 16);
            displayBuffer[16] = '\0';
            index++;
            if (index > messageLen - 16)
            {
                index = 0;
            }
        }

        lcd.setCursor(0, row);
        lcd.print(displayBuffer);
    }
}

void loadSettings()
{
    EEPROM.get(0, settings);
    if (settings.overVoltage < 100 || settings.overVoltage > 300)
    {
        settings = Settings(); // load defaults if invalid
        EEPROM.put(0, settings);
    }
}

void saveSettings()
{
    EEPROM.put(0, settings);
}

void showStatusScreen()
{

    lcd.clear();
    static bool alternateScreen = false;
    static unsigned long lastToggleTime = 0;
    const unsigned long toggleInterval = 1000; // 1 second
    switch (screenIndex)
    {
    case 0:
        lcd.setCursor(0, 0);
        lcd.print("V:");
        lcd.print(voltage);
        lcd.print(" I:");
        lcd.print(current);

        lcd.setCursor(0, 1);
        lcd.print("PF:");
        lcd.print(pf, 2); // PF with 2 decimal places
        lcd.print(" M:");
        lcd.print(motorRunning);
        break;

    case 1:
        lcd.setCursor(0, 0);
        lcd.print("Power:");
        lcd.print(power);
        lcd.print(" W");

        lcd.setCursor(0, 1);
        lcd.print("Energy: ");
        lcd.print(energy);
        break;

    case 2:
        lcd.setCursor(0, 0);
        lcd.print("UGT:");
        lcd.print(digitalRead(FLOAT_UGT_PIN) ? "OK" : "LOW");
        lcd.print(" OHT:");
        lcd.print(digitalRead(FLOAT_OHT_PIN) ? "OK" : "LOW");

        lcd.setCursor(0, 1);
        lcd.print(" Mode:");
        lcd.print(systemMode == 0 ? "AUTO" : systemMode == 1 ? "Manual"
                                                             : "Calib");

        break;

    case 3:
        lcd.setCursor(0, 0);
        Serial.print("System State: ");
        Serial.println(error);
        Serial.println();
        if (error >= 1)
        {
            if (error >= 2)
            {
                lcd.print("ERROR:");
            }
            else
            {
                lcd.print("System State: ");
            }
            lcd.setCursor(0, 1);
            if (error >= 3)
            {
                if (millis() - lastToggleTime >= toggleInterval)
                {
                    alternateScreen = !alternateScreen;
                    lastToggleTime = millis();

                    // lcd.setCursor(0, 1);
                    // lcd.print("                "); // Clear line

                    lcd.setCursor(0, 1);
                    if (alternateScreen)
                    {
                        lcd.print(errorMessage);
                    }
                    else
                    {
                        lcd.print("SET key resets  ");
                    }
                }
            }
            else
            {
                lcd.setCursor(0, 1);
                lcd.print(errorMessage);
            }

            // scrollMessage("Over current - SET key resets ", 1); // 0 = first line
        }
        else
        {
            if (motorRunning && settings.onTime > 0)
            {
                unsigned long elapsed = ((millis() - lastOnTime) / 1000); // in sec
                unsigned long remaining = settings.onTime * 60 - elapsed; // in sec
                lcd.print("ON Time Left:");
                lcd.setCursor(0, 1);
                // remaining > 600 ? remaining : remaining<0 ?0:remaining*60;
                lcd.print(remaining > 600 ? remaining / 60 : remaining < 0 ? 0
                                                                           : remaining);

                lcd.print(remaining > 600 ? " min" : " sec");
            }
            else if (!motorRunning && settings.offTime > 0 && !digitalRead(FLOAT_OHT_PIN))
            {
                unsigned long elapsed = ((millis() - lastOffTime) / 1000);
                unsigned long remaining = settings.offTime * 60 - elapsed;
                lcd.print("OFF Time Left:");
                lcd.setCursor(0, 1);
                lcd.print(remaining > 600 ? remaining / 60 : remaining < 0 ? 0
                                                                           : remaining);

                lcd.print(remaining > 600 ? " min" : " sec");
            }
            else
            {
                lcd.print("System Idle...");
                lcd.setCursor(0, 1);
                lcd.print("                ");
            }
            break;
        }
    }
}

void showMenu()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Menu mode:");
    lcd.setCursor(0, 1);
    char buffer[4];
    switch (menuIndex)
    {
    case 0:
        lcd.print("VOLT Detect: ");
        snprintf(buffer, sizeof(buffer), "%s", settings.detectVoltage == 1 ? "ON" : "OFF");
        lcd.print(buffer);
        break;
    case 1:
        lcd.print("Over Volt:");
        lcd.print(settings.overVoltage, 1);
        break;
    case 2:
        lcd.print("Under Volt:");
        lcd.print(settings.underVoltage, 1);
        break;
    case 3:
        lcd.print("AMP Detect: ");
        snprintf(buffer, sizeof(buffer), "%s", settings.detectCurrent == 1 ? "ON" : "OFF");
        lcd.print(buffer);
        break;
    case 4:
        lcd.print("Over Curr:");
        lcd.print(settings.overCurrent, 1);
        break;
    case 5:
        lcd.print("Under Curr:");
        lcd.print(settings.underCurrent, 1);
        break;
    case 6:
        lcd.print("Dry Detect: ");
        snprintf(buffer, sizeof(buffer), "%s", settings.dryRun == 1 ? "ON" : "OFF");
        lcd.print(buffer);
        break;
    case 7:
        lcd.print("Min PF:");
        lcd.print(settings.minPF, 2);
        break;
    case 8:
        lcd.print("Cylic Timer: ");
        snprintf(buffer, sizeof(buffer), "%s", settings.cyclicTimer == 1 ? "ON" : "OFF");
        lcd.print(buffer);
        break;
    case 9:
        lcd.print("ON Time:");
        lcd.print(settings.onTime);
        lcd.print(" min");
        break;
    case 10:
        lcd.print("OFF Time:");
        lcd.print(settings.offTime);
        lcd.print(" min");
        break;
    }
}

void onSetClick()
{
    Serial.print("Menu Index: ");
    Serial.print(menuIndex);
    Serial.print("  IN Menu: ");
    Serial.println(inMenu);
    if (!inMenu)
    {
        buttonPressStart = millis();
    }
    else
    {
        menuIndex++;
        if ((menuIndex == 1 || menuIndex == 2) && !settings.detectVoltage)
        {
            menuIndex = 3;
        }
        else if ((menuIndex == 4 || menuIndex == 5) && !settings.detectCurrent)
        {
            menuIndex = 6;
        }
        else if ((menuIndex == 7) && !settings.dryRun)
        {
            menuIndex = 8;
        }
        else if ((menuIndex == 9 || menuIndex == 10) && !settings.cyclicTimer)
        {
            menuIndex = 11;
        }

        if (menuIndex > totalMenuItems)
        {
            inMenu = false;
            menuIndex = 0;
            saveSettings();
            showStatusScreen();
        }
    }
    lastInteractionTime = millis();
}

void onUpClick()
{ // hide menus based on value
    // settings.detectVoltage(case 0==0)->hide case 1 and 2
    // settings.detectCurrent(case 3==0)->hide case 4 and 5
    // settings.dryRun(case 6==0)->hide case 7 and 8
    // settings.cyclicTimer(case 8)->hide case 9 and 10
    if (!inMenu)
        return;

    switch (menuIndex)
    {
    case 0:
        settings.detectVoltage = !settings.detectVoltage;
        break;
    case 1:
        settings.overVoltage += 1.0;
        break;
    case 2:
        settings.underVoltage += 1.0;
        break;
    case 3:
        settings.detectCurrent = !settings.detectCurrent;
        break;
    case 4:
        settings.overCurrent += 0.1;
        break;
    case 5:
        settings.underCurrent += 0.1;
        break;
    case 6:
        settings.dryRun = !settings.dryRun;
        break;
    case 7:
        settings.minPF += 0.01;
        break;
    case 8:
        settings.cyclicTimer = !settings.cyclicTimer;
        break;
    case 9:
        settings.onTime += 1;
        settings.onTime < 1 ? settings.onTime = 1 : settings.onTime;
        break;
    case 10:
        settings.offTime += 1;
        settings.offTime < 1 ? settings.offTime = 1 : settings.offTime;
        break;
    }

    lastInteractionTime = millis();
    showMenu();
}

void onDownClick()
{
    if (!inMenu)
        return;

    switch (menuIndex)
    {
    case 0:
        settings.detectVoltage = !settings.detectVoltage;
        break;
    case 1:
        settings.overVoltage -= 1.0;
        break;
    case 2:
        settings.underVoltage -= 1.0;
        break;
    case 3:
        settings.detectCurrent = !settings.detectCurrent;
        break;
    case 4:
        settings.overCurrent -= 0.1;
        break;
    case 5:
        settings.underCurrent -= 0.1;
        break;
    case 6:
        settings.dryRun = !settings.dryRun;
        break;
    case 7:
        settings.minPF -= 0.01;
        break;
    case 8:
        settings.cyclicTimer = !settings.cyclicTimer;
        break;
    case 9:
        settings.onTime -= 1;
        settings.onTime < 1 ? settings.onTime = 1 : settings.onTime;
        break;
    case 10:
        settings.offTime -= 1;
        settings.offTime < 1 ? settings.offTime = 1 : settings.offTime;
        break;
    }
    lastInteractionTime = millis();
    showMenu();
}

int checkSystemStatus()
{
    if ((voltage < settings.underVoltage || voltage > settings.overVoltage) && settings.detectVoltage)
    {
        if (voltage < settings.underVoltage)
        {
            strcpy(errorMessage, "LOW Voltage");
        }
        if (voltage > settings.overVoltage)
        {
            strcpy(errorMessage, "HIGH Voltage");
        }
        return 3; // Voltage out of range
    }
    if (motorRunning)
    {
        if ((current > settings.overCurrent) && settings.detectCurrent)
        {
            strcpy(errorMessage, "Over current");
            return 4; // Over current
        }

        if ((current < settings.underCurrent) && settings.detectCurrent)
        {
            strcpy(errorMessage, "Under current");
            return 5; // Under current
        }

        if (current < settings.underCurrent && pf < settings.minPF && settings.dryRun)
        {
            strcpy(errorMessage, "Dry run");
            return 6; // Dry run
        }
    }
    if (!digitalRead(FLOAT_UGT_PIN))
    {

        strcpy(errorMessage, "UGT empty");
        return 2; // UGT empty
    }
    if (!digitalRead(FLOAT_OHT_PIN))
    {
        strcpy(errorMessage, "OHT LOW");
        return 1; // OHT low
    }

    lasterrorTime = millis();
    return 0; // All OK
}

void blinkLED(int led)
{
    if (millis() - lastBlinkTime >= 500)
    {                         // 500ms blink interval
        ledState = !ledState; // toggle state
        digitalWrite(led, ledState);
        lastBlinkTime = millis();
    }
}

void buttonCheck()
{
    static int
        count,         // the number that is adjusted
        lastCount(-1); // previous value of count (initialized to ensure it's different when the sketch starts)
    static unsigned long
        rpt(REPEAT_FIRST); // a variable time that is used to drive the repeats for long presses
    enum states_t
    {
        WAIT,
        INCR,
        DECR,
        MENU
    }; // states for the state machine
    static states_t STATE; // current state machine state
    btnUP.read();          // read the buttons
    btnDN.read();
    btnSET.read();
    if (btnSET.wasPressed())
    {
        Serial.println("Set button pressed");
    }
    if (btnUP.wasPressed())
    {
        Serial.println("UP button pressed");
    }
    else if (btnDN.wasPressed())
    {
        Serial.println("DOWN button pressed");
    }

    if (count != lastCount) // print the count if it has changed
    {
        lastCount = count;
        Serial.println(count, DEC);
    }

    switch (STATE)
    {
    case WAIT: // wait for a button event
        if (btnSET.wasPressed())
            STATE = MENU;
        if (btnUP.wasPressed())
            STATE = INCR;
        else if (btnDN.wasPressed())
            STATE = DECR;
        else if (btnUP.wasReleased()) // reset the long press interval
            rpt = REPEAT_FIRST;
        else if (btnDN.wasReleased())
            rpt = REPEAT_FIRST;
        else if (btnUP.pressedFor(rpt)) // check for long press
        {
            rpt += REPEAT_INCR; // increment the long press interval
            STATE = INCR;
        }
        else if (btnDN.pressedFor(rpt))
        {
            rpt += REPEAT_INCR;
            STATE = DECR;
        }
        break;

    case INCR:
        ++count; // increment the counter
        onUpClick();
        count = min(count, MAX_COUNT); // but not more than the specified maximum
        STATE = WAIT;
        break;

    case DECR:
        --count; // decrement the counter
        onDownClick();
        count = max(count, MIN_COUNT); // but not less than the specified minimum
        STATE = WAIT;
        break;
    case MENU:
        printf("inMenu: %d", inMenu);
        printf(",  menuIndex: %d \n", menuIndex);
        showMenu();

        menuIndex++;
        if (inMenu)
        {
            if ((menuIndex == 1 || menuIndex == 2) && !settings.detectVoltage)
            {
                menuIndex = 3;
            }
            else if ((menuIndex == 4 || menuIndex == 5) && !settings.detectCurrent)
            {
                menuIndex = 6;
            }
            else if ((menuIndex == 7) && !settings.dryRun)
            {
                menuIndex = 8;
            }
            else if ((menuIndex == 9 || menuIndex == 10) && !settings.cyclicTimer)
            {
                menuIndex = 11;
            }

            if (menuIndex > totalMenuItems)
            {
                inMenu = false;
                menuIndex = 0;
                saveSettings();
                showStatusScreen();
            }
        }
        else
        {
            inMenu = true;
        }
        STATE = WAIT;
        break;
    }
}

void calibrateMotor()
{
    if (calibCancelled)
    {
        return;
    }
    systemMode = 2;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Calibrating.....");
    lcd.setCursor(0, 1);
    lcd.print("Waiting for intialize.");
    while (digitalRead(KEY_SET) == HIGH)
    {
        if (digitalRead(KEY_UP) == LOW || digitalRead(KEY_DOWN) == LOW)
        {
            lcd.setCursor(0, 1);
            lcd.print("Change Sw 2 AUTO");
            calibCancelled = true;
            return;
        }
    }
    digitalWrite(MOTOR_RELAY_PIN, HIGH);
    digitalWrite(MOTOR_STATUS_LED, HIGH);

    motorRunning = true;
    lastOnTime = millis();
    int currentSec = (millis() - lastOnTime) / 1000;
    while (millis() - lastOnTime < 20000)
    {
        currentSec = (millis() - lastOnTime) / 1000;
        lcd.setCursor(0, 1);
        lcd.print("Wait for ");
        lcd.print(20 - currentSec);
        lcd.print(" sec");
    }

    const int samples = 5;
    const unsigned long sampleDelay = 500; // ms between samples

    float sumV = 0, sumI = 0, sumPF = 0;

    Serial.println("Starting auto-calibration...");
    lcd.setCursor(0, 0);
    lcd.print("Starting Auto   ");
    lcd.setCursor(0, 1);
    lcd.print("     Calibration");

    for (int i = 0; i < samples; i++)
    {
        float v = pzem.voltage();
        float i_ = current; // pzem.current();
        float pf = pf;      // pzem.pf();

        if (isnan(v) || isnan(i_) || isnan(pf))
        {
            Serial.println("Error: Invalid PZEM reading (NaN)");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Error:");
            lcd.setCursor(0, 1);
            lcd.print("PZEM Reading ERR");
            digitalWrite(MOTOR_RELAY_PIN, LOW);
            digitalWrite(ERROR_LED, LOW);
            delay(500);
            return;
        }

        sumV += v;
        sumI += i_;
        sumPF += pf;
        delay(sampleDelay);
    }

    // Stop motor after calibration
    digitalWrite(MOTOR_RELAY_PIN, LOW);
    digitalWrite(ERROR_LED, LOW);

    lastOffTime = millis();
    // Compute averages
    voltage = sumV / samples;
    current = sumI / samples;
    pf = sumPF / samples;

    voltage = pzem.voltage();
    current = pzem.current();
    power = pzem.power();
    pf = pzem.pf();
    energy = pzem.energy();

    // Calculate with ±20% margins and validate
    settings.minPF = max(0.1, pf - pf * 0.2);
    settings.overCurrent = current + current * 0.2;
    settings.underCurrent = max(0.1, current - current * 0.2);
    settings.overVoltage = voltage + voltage * 0.2;
    settings.underVoltage = max(50.0, voltage - voltage * 0.2);
    settings.offTime = 1;
    settings.onTime = 1;
    // Save to EEPROM
    saveSettings();

    // Feedbackprintf("Calibration completed successfully:\n");
    printf("Min PF: %.2f\n", settings.minPF);
    printf("Over Current: %.2f A\n", settings.overCurrent);
    printf("Under Current: %.2f A\n", settings.underCurrent);
    printf("Over Voltage: %.1f V\n", settings.overVoltage);
    printf("Under Voltage: %.1f V\n", settings.underVoltage);

    // Serial.println("Calibration completed successfully:");
    // Serial.print("Min PF: ");
    // Serial.println(settings.minPF);
    // Serial.print("Over Current: ");
    // Serial.println(settings.overCurrent);
    // Serial.print("Under Current: ");
    // Serial.println(settings.underCurrent);
    // Serial.print("Over Voltage: ");
    // Serial.println(settings.overVoltage);
    // Serial.print("Under Voltage: ");
    // Serial.println(settings.underVoltage);

    while (digitalRead(SW_AUTO))
    {
        lcd.setCursor(0, 0);
        lcd.print("Setting Saved   ");
        lcd.setCursor(0, 1);
        lcd.print("Change Sw 2 AUTO");
        digitalWrite(MOTOR_RELAY_PIN, LOW);
        motorRunning = false;
        calibMode = 0;
    }
}

void readPzemValues()
{
    voltage = pzem.voltage();
    current = pzem.current();
    power = pzem.power();
    pf = pzem.pf();
    energy = pzem.energy();
}

// --------------------- Setup -------------------------
void setup()
{
    Serial.begin(115200);
    // PZEMSerial.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    // Serial2.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    Serial2.begin(9600);
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Water Ctrl Start");

    pinMode(MOTOR_RELAY_PIN, OUTPUT);
    pinMode(MOTOR_STATUS_LED, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);
    digitalWrite(MOTOR_RELAY_PIN, LOW);
    digitalWrite(MOTOR_STATUS_LED, LOW);
    digitalWrite(ERROR_LED, LOW);

    pinMode(SW_AUTO, INPUT_PULLUP);
    pinMode(SW_MANUAL, INPUT_PULLUP);
    pinMode(FLOAT_OHT_PIN, INPUT_PULLUP);
    pinMode(FLOAT_UGT_PIN, INPUT_PULLUP);
    // WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;

    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
    // wm.resetSettings();

    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    bool res;
    res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    // res = wm.autoConnect("AutoConnectAP", "password"); // password protected ap

    if (!res)
    {
        Serial.println("Failed to connect");
        // ESP.restart();
    }
    else
    {
        // if you get here you have connected to the WiFi
        Serial.println("connected...yeey :)");
    }

    btnSET.begin();
    btnUP.begin();
    btnDN.begin();

    loadSettings();
    printf("System Booted on ESP32\n");
}

// --------------------- Main Loop -------------------------
void loop()
{

    buttonCheck();
    if ((millis() - lastOnTime > 5000) && error <= 3)
    {
        error = checkSystemStatus();
    }

    if (error >= 3)
    {
        blinkLED(ERROR_LED);
    }
    else
    {
        digitalWrite(ERROR_LED, LOW);
    }

    if (!inMenu && millis() - lastScreenSwitch >= 5000)
    {
        // printGpioInputs();
        if (error >= 3)
            screenIndex = 3;
        else
        {
            if (millis() - lasterrorTime > 60 * 60 * 1000UL)
                error = 0;
            screenIndex = (screenIndex + 1) % 4;
        }
        lastScreenSwitch = millis();
        showStatusScreen();
        printf("V:%.2f I:%.2f PF:%.2f P:%.2f UGT:%d OHT:%d Motor:%d ERROR:%d\n",
               voltage,
               current,
               pf,
               power,
               digitalRead(FLOAT_UGT_PIN),
               digitalRead(FLOAT_OHT_PIN),
               digitalRead(MOTOR_RELAY_PIN),
               error);
    }

    if (!inMenu && millis() - lastPzemRead >= pzemReadInterval)
    {
        readPzemValues();
        lastPzemRead = millis();
    }

    if (isnan(energy))
        energy = 0.0;
    if (isnan(voltage))
        voltage = 0.0;
    if (isnan(current))
        current = 0.0;
    if (isnan(power))
        power = 0.0;
    if (isnan(pf))
        pf = 0.0;

    // Serial.print("V:");
    // Serial.print(voltage);
    // Serial.print(" I:");
    // Serial.print(current);
    // Serial.print(" PF:");
    // Serial.print(pf);
    // Serial.print(" P:");
    // Serial.print(power);
    // Serial.print(" UGT:");
    // Serial.print(digitalRead(FLOAT_UGT_PIN));
    // Serial.print(" OHT:");
    // Serial.print(digitalRead(FLOAT_OHT_PIN));
    // Serial.print(" Motor:");
    // Serial.print(digitalRead(MOTOR_RELAY_PIN));
    // Serial.print(" ERROR:");
    // Serial.println(error);

    if (digitalRead(SW_MANUAL) && digitalRead(SW_AUTO) && calibMode)
    {
        calibrateMotor();
    }
    else if (!digitalRead(SW_AUTO) && digitalRead(SW_MANUAL))
    {
        systemMode = 0;
        if (manulallyON)
        {
            digitalWrite(MOTOR_RELAY_PIN, LOW);
            motorRunning = false;
            lastOffTime = millis();
        }

        if (motorRunning)
        {
            if (settings.onTime < 1)
            {
                settings.onTime = 10;
            }

            // Serial.print("millis() - lastOnTime (s): ");
            // Serial.print((millis() - lastOnTime) / 1000);
            // Serial.print(", Remaining Time to OFF (s): ");
            // Serial.println((settings.onTime * 60) - (millis() - lastOnTime) / 1000);
            if ((settings.cyclicTimer ? millis() - lastOnTime >= settings.onTime * 60000UL : 0) || error >= 2)
            {
                Serial.println("More than 60sec");
                digitalWrite(MOTOR_RELAY_PIN, LOW);
                digitalWrite(MOTOR_STATUS_LED, LOW);
                motorRunning = false;
                lastOffTime = millis();
            }
        }
        else
        {
            if (settings.offTime < 1)
            {
                settings.offTime = 10;
            }
            if (error == 1)
            {
                blinkLED(MOTOR_STATUS_LED);
            }

            // Serial.print("millis() - lastOffTime (s): ");
            // Serial.print((millis() - lastOffTime) / 1000);
            // Serial.print(", Remaining Time to ON (s): ");
            // Serial.println((settings.offTime * 60) - (millis() - lastOffTime) / 1000);
            if (settings.cyclicTimer ? millis() - lastOffTime >= settings.offTime * 60000UL : 1 && error == 1)
            {
                digitalWrite(MOTOR_RELAY_PIN, HIGH);
                motorRunning = true;
                lastOnTime = millis();
                digitalWrite(MOTOR_STATUS_LED, HIGH);
            }
            else if (error >= 2)
            {
                digitalWrite(MOTOR_STATUS_LED, LOW);
            }
        }
    }
    else if (!digitalRead(SW_MANUAL) && digitalRead(SW_AUTO))
    {
        systemMode = 1;
        if (error == 1)
        {
            digitalWrite(MOTOR_RELAY_PIN, HIGH);
            motorRunning = true;
            manulallyON = true;
            lastOnTime = millis();
        }
        else
        {
            digitalWrite(MOTOR_RELAY_PIN, LOW);
            motorRunning = false;
            lastOffTime = millis();
        }
    }
    else
    {
        lcd.setCursor(0, 0);
        lcd.print("System in Calib");

        lcd.setCursor(0, 1);
        lcd.print("Change Sw 2 AUTO");
    }
}
