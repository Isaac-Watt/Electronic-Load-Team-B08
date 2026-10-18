// ============================================================
// PROGRAMMABLE DC ELECTRONIC LOAD
// Freenove ESP32-WROOM + ST7796 480x320 TFT (landscape)
//
// TFT pins are set in platformio.ini build_flags:
//   MOSI 23, SCLK 18, CS 5, DC/RS 17, RST 16
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>

TFT_eSPI tft = TFT_eSPI();

// ============================================================
// PIN ASSIGNMENTS (ESP32-WROOM)
// ============================================================

// Buttons: wire each between the GPIO and GND
#define BTN_UP      25
#define BTN_DOWN    26
#define BTN_BACK    27   // "LEFT"
#define BTN_SELECT  14   // "RIGHT"

// Sensors: ADC1 pins (still work with WiFi on)
#define VOLTAGE_ADC_PIN     33
#define CURRENT_ADC_PIN     35   // input-only
// Thermistor pins are set in the THERMISTORS table below.

// Control reference output (PWM)
#define CONTROL_PIN         13

// Fan tach input = the fan's BLUE wire (labelled FOO). -1 = no fan fitted.
//   Blue -> GPIO22, plus a 10k resistor from GPIO22 to 3.3 V.
//   Fan black (-) must share ground with the ESP32.
// NEVER pull the tach up to 12 V - that would damage the ESP32.
#define FAN_TACH_PIN        22
const int FAN_PULSES_PER_REV = 2;   // standard PC fans give 2 pulses per turn
const float FAN_MAX_RPM      = 13300.0;   // Delta PFR0812XHE: 13,300 RPM at 12 V

// Shortest real gap between tach pulses at full speed:
//   13000 RPM x 2 pulses = 433 pulses/s -> 2.3 ms apart.
// Anything faster than 1.5x that is treated as noise and ignored.
const unsigned long FAN_MIN_PULSE_US =
    (unsigned long)(60000000.0 / (FAN_MAX_RPM * FAN_PULSES_PER_REV * 1.5));

// Pins NOT to use on this board:
//   0, 2, 12, 15  boot strapping pins
//   1, 3          USB serial (upload / monitor)
//   6-11          internal flash
//   5,16,17,18,23 TFT;  19 reserved as TFT MISO

// ============================================================
// SYSTEM LIMITS
// ============================================================

const float MAX_VOLTAGE = 30.0;
const float MAX_CURRENT = 30.0;
const float MAX_POWER   = 500.0;

// Fault limits: slight margin above the 30 V / 30 A / 500 W ratings
const float VOLTAGE_FAULT_LIMIT     = 30.5;
const float CURRENT_FAULT_LIMIT     = 30.5;
const float POWER_FAULT_LIMIT       = 510.0;
const float INT_TEMP_FAULT_LIMIT    = 70.0;   // internal (FETs / board): keep below 70 C
const float EXT_TEMP_FAULT_LIMIT    = 60.0;   // external battery probes

// ============================================================
// SENSOR SETTINGS
// ============================================================

const int ADC_SAMPLES = 64;

// Voltage divider: input -> 10k -> ADC pin -> 1k -> GND
// Pin voltage = Vin x 1k / (10k + 1k), so Vin = pin voltage x 11.
// 30 V in -> 2.73 V at the pin (safely under 3.3 V).
const float DIVIDER_R_TOP    = 10000.0;
const float DIVIDER_R_BOTTOM = 1000.0;
const float VOLTAGE_RATIO    = (DIVIDER_R_TOP + DIVIDER_R_BOTTOM) / DIVIDER_R_BOTTOM;

// Fine trim: after checking against a multimeter, set to
// (multimeter reading / screen reading). 1.0 = no correction.
const float VOLTAGE_TRIM = 1.0;

// A fault must be seen on this many readings in a row before it
// latches, so a single noisy sample can't trip the load.
const int FAULT_CONFIRM_COUNT = 3;

// Print raw sensor values to the serial monitor (for debugging)
const bool SERIAL_DEBUG = true;

// CSV logging over USB serial while a test is running.
// Lines start with "HDR," (column names), "DATA," (one reading) and
// "END," (test finished/stopped/faulted). The log_capture.py script
// on the laptop collects these and ignores everything else.
//
// When a test ends the screen asks "SAVE LOG TO CSV?":
//   RIGHT = YES -> sends "SAVE"    -> script writes the .csv and replies
//                                     "SAVED,<filename>" (shown on screen)
//   LEFT  = NO  -> sends "DISCARD" -> script throws that test's rows away
const bool CSV_LOGGING = true;

// CSV_ONLY = false: lines have HDR/DATA/END prefixes for log_capture.py
// (needed for the save prompt). true = bare CSV only, for saving with
// PlatformIO's log2file monitor instead (the save prompt then has no
// script to talk to and will report "no reply from PC").
const bool CSV_ONLY = false;

// All non-CSV messages go through "Info". In CSV_ONLY mode it's a dummy
// output that throws text away, so it never ends up in the CSV file.
class NullPrint : public Print {
public:
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t *, size_t n) override { return n; }
};
NullPrint nullPrint;
Print &Info = CSV_ONLY ? (Print &)nullPrint : (Print &)Serial;
const unsigned long LOG_INTERVAL_MS = 1000;   // one row per second
unsigned long lastLogTime = 0;

// ACS712: zero measured automatically at startup
float currentZeroVoltage = 1.70;
const float CURRENT_SENSITIVITY = 0.033;   // V/A, calibrate later
const float CURRENT_DEADBAND    = 0.10;

// Set to false while the ACS712 is NOT wired up. An unconnected pin
// floats and reads random voltages, which shows up as huge currents
// and trips the over-current fault. When false, current reads 0 A.
const bool CURRENT_SENSOR_CONNECTED = false;

// ------------------------------------------------------------
// THERMISTORS: 4 internal + 4 external, 10k NTC, Beta 3950
//
// Each channel is a divider:  3.3 V -> NTC -> ADC pin -> R_series -> GND
//   Internal: R_series = 1.8k  (most accurate around 78 C)
//   External: R_series = 10k   (most accurate around 25 C)
//
// pin = -1 means "not fitted": the channel is skipped and ignored.
// Only give a channel a pin once a thermistor is actually wired to it:
// an unconnected pin floats and would show a random temperature.
//
// The Freenove ESP32-WROOM only has a few free ADC pins, so only 3
// channels can be wired on it. On the main ESP32-S3 PCB all 8 go on
// IO1-IO8 (set the pins to match the schematic).
// ------------------------------------------------------------
struct Thermistor {
    const char *name;
    int   pin;
    float seriesR;   // fixed resistor in the divider (ohms)
    bool  internal;
    float r25;       // thermistor resistance at 25 C (ohms) - MEASURE IT
    float beta;      // from the thermistor datasheet (3950 is common)
};

Thermistor THERMISTORS[8] = {
    // name    pin   R_series  internal   R25      Beta
    { "INT1",    4,  10000.0,  true,     10000.0, 3950.0 },   // GPIO4 (ADC2: WiFi off)
                                                               // bench test: 10k resistor.
                                                               // Change to 1800.0 once a 1.8k is fitted.
    { "INT2",   -1,   1800.0,  true,     10000.0, 3950.0 },   // GPIO34 suspect (had 5V)
    { "INT3",   -1,   1800.0,  true,     10000.0, 3950.0 },
    { "INT4",   -1,   1800.0,  true,     10000.0, 3950.0 },
    { "EXT1",   32,  10000.0,  false,    10000.0, 3950.0 },
    { "EXT2",   -1,  10000.0,  false,    10000.0, 3950.0 },
    { "EXT3",   -1,  10000.0,  false,    10000.0, 3950.0 },
    { "EXT4",   -1,  10000.0,  false,    10000.0, 3950.0 },
};
const int NUM_THERMISTORS = 8;

const float ADC_SUPPLY = 3.3;

// true  = NTC on top    (3.3 V -> NTC -> pin -> R -> GND)
// false = NTC on bottom (3.3 V -> R -> pin -> NTC -> GND)
// If a reading goes DOWN when you warm the thermistor, flip this.
const bool NTC_ON_TOP = false;

// true = any fitted thermistor that is open/shorted trips a fault.
// Set false for testing with thermistors unplugged.
const bool THERMISTOR_REQUIRED = true;

// Shown on screen for a thermistor that isn't connected (open/short or
// not fitted). Display only: the fault logic still sees it as missing.
const float TEMP_DISCONNECTED = 6.66;

// ============================================================
// STATES
// ============================================================

enum LoadMode { MODE_CC, MODE_CV };
LoadMode loadMode = MODE_CC;

enum Screen {
    SCREEN_MAIN, SCREEN_MODE, SCREEN_CURRENT, SCREEN_VOLTAGE,
    SCREEN_DURATION, SCREEN_CONFIRM, SCREEN_INVALID,
    SCREEN_RUNNING, SCREEN_COMPLETE, SCREEN_FAULT,
    SCREEN_SAVE_PROMPT, SCREEN_SAVING, SCREEN_CUTOFF
};
Screen screen = SCREEN_MAIN;

enum FaultCode {
    FAULT_NONE, FAULT_OVERVOLTAGE, FAULT_OVERCURRENT,
    FAULT_OVERPOWER, FAULT_OVERTEMP, FAULT_BATTERY_TEMP, FAULT_SENSOR
};
int faultChannel = -1;   // which thermistor caused a temperature fault
FaultCode activeFault = FAULT_NONE;

enum ConfigError {
    CONFIG_OK, CONFIG_CURRENT_LIMIT, CONFIG_VOLTAGE_LIMIT, CONFIG_POWER_LIMIT,
    CONFIG_BELOW_CUTOFF
};
ConfigError configurationError = CONFIG_OK;

// ============================================================
// MENU / SETTINGS
// ============================================================

int menuIndex = 0;
int modeIndex = 0;

float currentSetpoint = 1.00;
float voltageSetpoint = 5.00;
int durationMinutes = 10;

float currentSteps[] = { 0.01, 0.10, 1.00 };
int currentStepIndex = 0;

float voltageSteps[] = { 0.10, 1.00, 5.00 };
int voltageStepIndex = 0;

int timeSteps[] = { 1, 5, 10, 30 };
int timeStepIndex = 0;

// ------------------------------------------------------------
// BATTERY CUTOFFS (end the test normally, not a fault)
//   CC mode: cutoff VOLTAGE = minimum battery voltage. The test stops
//            when the battery falls below it (battery is "empty").
//   CV mode: cutoff CURRENT = maximum discharge current. The test stops
//            when the current rises above it.
// 0 = OFF. The condition must last CUTOFF_DELAY_MS so a brief dip or
// noise spike doesn't end the test.
// ------------------------------------------------------------
float cutoffVoltage = 0.0;     // V, CC mode
float cutoffCurrent = 0.0;     // A, CV mode
int cutoffVoltageStepIndex = 0;
int cutoffCurrentStepIndex = 0;
const unsigned long CUTOFF_DELAY_MS = 500;

// ============================================================
// LIVE MEASUREMENTS
// ============================================================

float measuredVoltage = 0.0;
float measuredCurrent = 0.0;
float measuredPower = 0.0;
float thermTemp[8];              // each channel, NAN if open/short/not fitted
float thermPinV[8];              // raw pin voltage per channel (for debugging)
float maxIntTemp = NAN;          // MAX INT Temp
float maxExtTemp = NAN;          // MAX EXT Temp

volatile uint32_t fanPulses = 0; // counted by the tach interrupt
float fanRpm = 0.0;

// Raw pin voltages (for serial debug)
float lastVoltagePinV = 0.0;
float lastCurrentPinV = 0.0;

// ============================================================
// TIMER
// ============================================================

bool testRunning = false;
unsigned long testStartTime = 0;
unsigned long testDurationMs = 0;
unsigned long remainingMs = 0;

// Save-to-CSV prompt shown after every test (complete, stopped or fault)
bool savePromptPending = false;          // a test ended and hasn't been answered
char lastEndReason[24] = "";             // shown on the prompt screen
unsigned long saveRequestTime = 0;       // when "SAVE" was sent
const unsigned long SAVE_REPLY_TIMEOUT_MS = 3000;
enum SaveResult { SAVE_WAITING, SAVE_OK, SAVE_NO_REPLY };
SaveResult saveResult = SAVE_WAITING;
char savedFileName[48] = "";

// ============================================================
// NEAR-LIMIT WARNINGS
//
// Warns the user (orange banner on the main menu and running screen,
// plus a "warning" column in the CSV log) when a protected value comes
// within WARNING_PERCENT of its fault limit. The load keeps running;
// the fault itself still trips at the limit as before.
// ============================================================
const float WARNING_PERCENT    = 10.0;   // warn at 90 % of a fault limit
const float WARNING_HYSTERESIS = 2.0;    // clears below 88 % so it doesn't flicker

enum WarnItem { WARN_VOLTAGE, WARN_CURRENT, WARN_POWER, WARN_INT_TEMP, WARN_EXT_TEMP, WARN_COUNT };
const char *WARN_NAMES[WARN_COUNT] = { "VOLTAGE", "CURRENT", "POWER", "INT TEMP", "EXT TEMP" };
bool  warnActive[WARN_COUNT]  = { false };
float warnPercent[WARN_COUNT] = { 0 };
int   worstWarning = -1;                 // highest active warning, -1 = none

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 250;

// ============================================================
// BUTTON DEBOUNCE
// ============================================================

const unsigned long BUTTON_DEBOUNCE = 180;
unsigned long lastUpPress = 0;
unsigned long lastDownPress = 0;
unsigned long lastBackPress = 0;
unsigned long lastSelectPress = 0;

float calculatedConfigPower = 0.0;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void redraw();
void drawHeader(String title);
void drawMainScreen();
void drawModeScreen();
void drawCurrentScreen();
void drawVoltageScreen();
void drawDurationScreen();
void drawConfirmScreen();
void drawInvalidScreen();
void drawRunningScreen();
void updateRunningValues();
void updateMainVoltage();
void updateFanSpeed();
void logCsvHeader();
void logCsvRow();
void logCsvEnd(const char *reason);
void drawCompleteScreen();
void drawFaultScreen();
void drawSavePromptScreen();
void drawCutoffScreen();
void checkCutoff();
void updateWarnings();
void updateWarningBanner(bool force);
void drawSavingScreen();
void checkSaveReply();

void handleMenu();
bool buttonPressed(int pin, unsigned long &lastPress);

float readADCVoltageRaw(int pin);
void readMeasurements();
float readVoltage();
float readCurrent();
float readThermistor(int channel);
void readTemperatures();
void calibrateCurrentZero();

FaultCode detectFault();
void checkFaults();
bool configurationValid();

void startTest();
void stopTest();
void updateTimer();

void enableControl();
void disableControl();
void updateControlSignal();

// ============================================================
// FAN SPEED
// ============================================================

// Blue wire (FOO / tach): the fan pulls it low twice per revolution.
// Pulses closer together than FAN_MIN_PULSE_US (~1.5 ms) can't come
// from a 13000 RPM fan, so they're electrical noise and are ignored.
volatile unsigned long lastTachMicros = 0;

void IRAM_ATTR fanTachISR()
{
    // The ESP32 can fire a "falling" interrupt on noise or even on a
    // rising edge. Only count it if the line has genuinely gone LOW and
    // stayed low for a few microseconds.
    if (digitalRead(FAN_TACH_PIN) != LOW)
        return;
    delayMicroseconds(5);
    if (digitalRead(FAN_TACH_PIN) != LOW)
        return;

    unsigned long now = micros();
    if (now - lastTachMicros >= FAN_MIN_PULSE_US)
    {
        fanPulses++;
        lastTachMicros = now;
    }
}

// Converts tach pulses to RPM once per second
void updateFanSpeed()
{
    static unsigned long lastCalc = 0;
    unsigned long now = millis();

    if (FAN_TACH_PIN < 0 || now - lastCalc < 1000)
        return;

    noInterrupts();
    uint32_t pulses = fanPulses;
    fanPulses = 0;
    interrupts();

    unsigned long elapsed = now - lastCalc;
    lastCalc = now;

    fanRpm = (pulses * 60000.0) / (elapsed * FAN_PULSES_PER_REV);

    // Readings well above the fan's maximum can only be noise
    if (fanRpm > FAN_MAX_RPM * 1.3)
        Info.printf("Fan tach noise: %.0f RPM (check pull-up, capacitor, ground)\n", fanRpm);
}

// ============================================================
// CSV LOGGING (over USB serial)
// ============================================================

void logCsvHeader()
{
    if (!CSV_LOGGING) return;

    Serial.print(CSV_ONLY ? "" : "HDR,");
    Serial.print("time_s,mode,setpoint,voltage_V,current_A,power_W");
    for (int i = 0; i < NUM_THERMISTORS; i++)
    {
        if (THERMISTORS[i].pin >= 0)
            Serial.printf(",%s_C", THERMISTORS[i].name);
    }
    Serial.println(",max_int_C,max_ext_C,fan_rpm,warning");
}

// Writes a number, or leaves the cell empty if there's no reading
static void csvTemp(float t)
{
    if (isnan(t)) Serial.print(",");
    else          Serial.printf(",%.1f", t);
}

void logCsvRow()
{
    if (!CSV_LOGGING) return;

    float t = (millis() - testStartTime) / 1000.0;
    float setpoint = (loadMode == MODE_CC) ? currentSetpoint : voltageSetpoint;

    Serial.printf("%s%.1f,%s,%.2f,%.3f,%.3f,%.2f",
                  CSV_ONLY ? "" : "DATA,", t, loadMode == MODE_CC ? "CC" : "CV", setpoint,
                  measuredVoltage, measuredCurrent, measuredPower);

    for (int i = 0; i < NUM_THERMISTORS; i++)
    {
        if (THERMISTORS[i].pin >= 0)
            csvTemp(thermTemp[i]);
    }
    csvTemp(maxIntTemp);
    csvTemp(maxExtTemp);
    Serial.printf(",%.0f,%s\n", fanRpm, worstWarning >= 0 ? WARN_NAMES[worstWarning] : "");
}

void logCsvEnd(const char *reason)
{
    // Every test end leads to the "save log?" prompt
    strncpy(lastEndReason, reason, sizeof(lastEndReason) - 1);
    lastEndReason[sizeof(lastEndReason) - 1] = '\0';
    savePromptPending = CSV_LOGGING;

    if (!CSV_LOGGING || CSV_ONLY) return;   // plain-CSV mode: no END line
    Serial.printf("END,%.1f,%s\n", (millis() - testStartTime) / 1000.0, reason);
}

// Reads the laptop script's reply ("SAVED,<filename>") after "SAVE" was sent
void checkSaveReply()
{
    static char line[64];
    static int len = 0;

    while (Serial.available())
    {
        char c = Serial.read();
        if (c != '\r' && c != '\n')
        {
            if (len < (int)sizeof(line) - 1) line[len++] = c;
            continue;
        }
        if (len == 0) continue;
        line[len] = '\0';
        len = 0;

        if (strncmp(line, "SAVED,", 6) == 0)
        {
            strncpy(savedFileName, line + 6, sizeof(savedFileName) - 1);
            savedFileName[sizeof(savedFileName) - 1] = '\0';
            saveResult = SAVE_OK;
            redraw();
            return;
        }
    }

    if (saveResult == SAVE_WAITING && millis() - saveRequestTime > SAVE_REPLY_TIMEOUT_MS)
    {
        saveResult = SAVE_NO_REPLY;
        redraw();
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Info.println();
    Info.println("====================================");
    Info.println("PROGRAMMABLE DC ELECTRONIC LOAD");
    Info.println("====================================");

    // Buttons: internal pull-ups (harmless if external pull-ups are fitted)
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_BACK, INPUT_PULLUP);
    pinMode(BTN_SELECT, INPUT_PULLUP);

    // Control output
    pinMode(CONTROL_PIN, OUTPUT);
    disableControl();

    // ADC
    analogReadResolution(12);
    analogSetPinAttenuation(VOLTAGE_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(CURRENT_ADC_PIN, ADC_11db);
    for (int i = 0; i < NUM_THERMISTORS; i++)
    {
        if (THERMISTORS[i].pin >= 0)
            analogSetPinAttenuation(THERMISTORS[i].pin, ADC_11db);
    }

    // Current sensor auto-zero: NO current must be flowing at startup
    if (CURRENT_SENSOR_CONNECTED)
    {
        calibrateCurrentZero();
    }

    // Fan tach
    if (FAN_TACH_PIN >= 0)
    {
        pinMode(FAN_TACH_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), fanTachISR, FALLING);
    }

    // TFT
    tft.init();
    tft.setRotation(1);   // landscape
    tft.fillScreen(TFT_BLACK);

    redraw();

    Info.println();
    Info.println("System initialised.");
    Info.println("Pin map:");
    Info.println("  Buttons: UP=25 DOWN=26 BACK=27 SELECT=14");
    Info.printf("  Voltage ADC = GPIO%d\n", VOLTAGE_ADC_PIN);
    Info.println("  Current ADC = GPIO35");
    for (int i = 0; i < NUM_THERMISTORS; i++)
    {
        if (THERMISTORS[i].pin >= 0)
            Info.printf("  %s = GPIO%d\n", THERMISTORS[i].name, THERMISTORS[i].pin);
    }
    Info.println("  Control Out = GPIO13");
    Info.printf("  Fan tach (blue wire) = GPIO%d\n", FAN_TACH_PIN);
    Info.print("ACS712 zero voltage = ");
    Info.print(currentZeroVoltage, 4);
    Info.println(" V");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    readMeasurements();
    updateFanSpeed();
    checkFaults();
    updateWarnings();

    // Serial debug: raw pin voltages and what they convert to
    static unsigned long lastDebug = 0;
    if (SERIAL_DEBUG && millis() - lastDebug >= 500)
    {
        lastDebug = millis();
        Info.printf("V pin %.3f V -> %.2f V | I pin %.3f V (zero %.3f) -> %.2f A | MAX INT %.1f C | MAX EXT %.1f C\n",
                      lastVoltagePinV, measuredVoltage,
                      lastCurrentPinV, currentZeroVoltage, measuredCurrent,
                      maxIntTemp, maxExtTemp);

        Info.print("   Temps:");
        for (int i = 0; i < NUM_THERMISTORS; i++)
        {
            if (THERMISTORS[i].pin < 0) continue;
            if (isnan(thermTemp[i]))
                Info.printf("  %s=%.2fC(not connected)", THERMISTORS[i].name, TEMP_DISCONNECTED);
            else
                Info.printf("  %s=%.1fC (pin %.2fV)", THERMISTORS[i].name, thermTemp[i], thermPinV[i]);
        }
        Info.printf("  FAN=%.0f RPM\n", fanRpm);
    }

    // Fault screen has highest priority
    if (activeFault != FAULT_NONE)
    {
        disableControl();
        testRunning = false;

        if (screen != SCREEN_FAULT)
        {
            screen = SCREEN_FAULT;
            redraw();
        }

        handleMenu();
        return;
    }

    // Live voltage on the main menu
    if (screen == SCREEN_MAIN &&
        millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)
    {
        lastDisplayUpdate = millis();
        updateMainVoltage();
        updateWarningBanner(false);
    }

    if (screen == SCREEN_RUNNING)
    {
        updateTimer();

        if (screen == SCREEN_RUNNING)
            checkCutoff();

        if (screen == SCREEN_RUNNING)
        {
            updateControlSignal();

            if (millis() - lastLogTime >= LOG_INTERVAL_MS)
            {
                lastLogTime = millis();
                logCsvRow();
            }

            if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)
            {
                lastDisplayUpdate = millis();
                updateRunningValues();
                updateWarningBanner(false);
            }
        }
    }

    handleMenu();
}

// ============================================================
// SENSORS
// ============================================================

// Uses count / 4095 * 3.3 because the voltage calibration was
// originally determined this way.
float readADCVoltageRaw(int pin)
{
    uint32_t total = 0;

    // Throw away the first reading after switching channel: the ADC's
    // sample capacitor still holds charge from the previous pin, which
    // otherwise leaks one sensor's voltage into the next reading.
    analogRead(pin);
    delayMicroseconds(50);

    for (int i = 0; i < ADC_SAMPLES; i++)
    {
        total += analogRead(pin);
    }

    float average = total / (float)ADC_SAMPLES;
    return (average / 4095.0) * 3.3;
}

void calibrateCurrentZero()
{
    Info.println();
    Info.println("Calibrating current sensor zero...");
    Info.println("Ensure NO current is flowing.");

    delay(500);

    const int zeroSamples = 200;
    float total = 0.0;

    for (int i = 0; i < zeroSamples; i++)
    {
        total += readADCVoltageRaw(CURRENT_ADC_PIN);
        delay(3);
    }

    currentZeroVoltage = total / zeroSamples;

    Info.print("Current sensor zero = ");
    Info.print(currentZeroVoltage, 4);
    Info.println(" V");
}

void readMeasurements()
{
    measuredVoltage = readVoltage();
    measuredCurrent = readCurrent();
    readTemperatures();
    measuredPower = measuredVoltage * measuredCurrent;
}

// Averaged pin voltage using the ESP32's factory ADC calibration
float readPinVoltage(int pin)
{
    analogRead(pin);              // discard first reading after switching
    delayMicroseconds(50);

    uint32_t total = 0;
    for (int i = 0; i < ADC_SAMPLES; i++)
    {
        total += analogReadMilliVolts(pin);
    }

    return (total / (float)ADC_SAMPLES) / 1000.0;
}

float readVoltage()
{
    float pinV = readPinVoltage(VOLTAGE_ADC_PIN);

    // Smoothing: each new reading only moves the value 20% of the way,
    // which removes jitter (settles in roughly 50-100 ms).
    static bool first = true;
    if (first)
    {
        lastVoltagePinV = pinV;
        first = false;
    }
    else
    {
        lastVoltagePinV = 0.8 * lastVoltagePinV + 0.2 * pinV;
    }

    float actualVoltage = lastVoltagePinV * VOLTAGE_RATIO * VOLTAGE_TRIM;

    // ESP32 ADC can't read the last ~0.1 V at the pin; treat as 0
    if (actualVoltage < 0.5)
    {
        actualVoltage = 0.0;
    }

    return actualVoltage;
}

float readCurrent()
{
    if (!CURRENT_SENSOR_CONNECTED)
    {
        lastCurrentPinV = 0.0;
        return 0.0;
    }

    float adcVoltage = readADCVoltageRaw(CURRENT_ADC_PIN);
    lastCurrentPinV = adcVoltage;
    float current = fabs((adcVoltage - currentZeroVoltage) / CURRENT_SENSITIVITY);

    if (current < CURRENT_DEADBAND)
    {
        current = 0.0;
    }

    return current;
}

// One thermistor channel -> degrees C (NAN if open/short or not fitted)
float readThermistor(int channel)
{
    const Thermistor &t = THERMISTORS[channel];
    if (t.pin < 0)
        return NAN;

    analogRead(t.pin);                 // discard first reading after switching
    delayMicroseconds(50);

    uint32_t total = 0;
    const int samples = 16;
    for (int i = 0; i < samples; i++)
    {
        total += analogReadMilliVolts(t.pin);
    }
    float voltage = (total / (float)samples) / 1000.0;
    thermPinV[channel] = voltage;

    // Open or short circuit (pin pinned to a rail)
    // Within 0.15 V of either rail = open or shorted thermistor
    if (voltage <= 0.15 || voltage >= ADC_SUPPLY - 0.15)
    {
        return NAN;
    }

    float ntcR;
    if (NTC_ON_TOP)
        ntcR = t.seriesR * (ADC_SUPPLY - voltage) / voltage;
    else
        ntcR = t.seriesR * voltage / (ADC_SUPPLY - voltage);

    const float T0 = 25.0 + 273.15;
    float kelvin = 1.0 / ((1.0 / T0) + (1.0 / t.beta) * log(ntcR / t.r25));

    return kelvin - 273.15;
}

// Read all channels and work out MAX INT Temp / MAX EXT Temp
void readTemperatures()
{
    maxIntTemp = NAN;
    maxExtTemp = NAN;

    for (int i = 0; i < NUM_THERMISTORS; i++)
    {
        thermTemp[i] = readThermistor(i);
        if (isnan(thermTemp[i]))
            continue;

        if (THERMISTORS[i].internal)
        {
            if (isnan(maxIntTemp) || thermTemp[i] > maxIntTemp)
                maxIntTemp = thermTemp[i];
        }
        else
        {
            if (isnan(maxExtTemp) || thermTemp[i] > maxExtTemp)
                maxExtTemp = thermTemp[i];
        }
    }
}

// "45.2 C", or the "disconnected" value if no working sensor
void formatTemp(char *buffer, float temp)
{
    if (isnan(temp))
        sprintf(buffer, "%.2f C", TEMP_DISCONNECTED);
    else
        sprintf(buffer, "%.1f C", temp);
}

// ============================================================
// BUTTONS
// ============================================================

// Triggers once per press (on the moment it goes down), not
// repeatedly while held. A stuck/miswired button therefore can't
// spam redraws.
bool buttonPressed(int pin, unsigned long &lastPress)
{
    static bool wasDown[40] = { false };

    bool down = (digitalRead(pin) == LOW);
    bool pressed = false;

    if (down && !wasDown[pin] && millis() - lastPress > BUTTON_DEBOUNCE)
    {
        lastPress = millis();
        pressed = true;
    }

    wasDown[pin] = down;
    return pressed;
}

// ============================================================
// FAULTS
// ============================================================

FaultCode detectFault()
{
    for (int i = 0; i < NUM_THERMISTORS; i++)
    {
        if (THERMISTORS[i].pin < 0)
            continue;

        if (THERMISTOR_REQUIRED && isnan(thermTemp[i]))
        {
            faultChannel = i;
            return FAULT_SENSOR;
        }

        if (isnan(thermTemp[i]))
            continue;

        if (THERMISTORS[i].internal && thermTemp[i] > INT_TEMP_FAULT_LIMIT)
        {
            faultChannel = i;
            return FAULT_OVERTEMP;
        }

        if (!THERMISTORS[i].internal && thermTemp[i] > EXT_TEMP_FAULT_LIMIT)
        {
            faultChannel = i;
            return FAULT_BATTERY_TEMP;
        }
    }

    if (measuredVoltage > VOLTAGE_FAULT_LIMIT)
        return FAULT_OVERVOLTAGE;

    if (measuredCurrent > CURRENT_FAULT_LIMIT)
        return FAULT_OVERCURRENT;

    if (measuredPower > POWER_FAULT_LIMIT)
        return FAULT_OVERPOWER;

    return FAULT_NONE;
}

// Latch new faults
void checkFaults()
{
    static int faultCount = 0;

    if (activeFault != FAULT_NONE)
        return;

    FaultCode detected = detectFault();

    if (detected == FAULT_NONE)
    {
        faultCount = 0;
        return;
    }

    faultCount++;

    if (faultCount >= FAULT_CONFIRM_COUNT)
    {
        faultCount = 0;
        activeFault = detected;
        disableControl();

        if (testRunning)
        {
            logCsvRow();
            char reason[24];
            sprintf(reason, "FAULT_%d", (int)detected);
            logCsvEnd(reason);
        }
        testRunning = false;

        Info.print("FAULT latched: code ");
        Info.println((int)detected);
    }
}

// ============================================================
// CONFIGURATION VALIDATION
// ============================================================

bool configurationValid()
{
    configurationError = CONFIG_OK;
    calculatedConfigPower = 0.0;

    if (currentSetpoint > MAX_CURRENT)
    {
        configurationError = CONFIG_CURRENT_LIMIT;
        return false;
    }

    if (voltageSetpoint > MAX_VOLTAGE)
    {
        configurationError = CONFIG_VOLTAGE_LIMIT;
        return false;
    }

    // CC: battery already at/below the cutoff voltage -> nothing to discharge
    if (loadMode == MODE_CC && cutoffVoltage > 0.0 && measuredVoltage <= cutoffVoltage)
    {
        configurationError = CONFIG_BELOW_CUTOFF;
        return false;
    }

    // CC power check: measured DUT voltage x requested current
    if (loadMode == MODE_CC)
    {
        calculatedConfigPower = measuredVoltage * currentSetpoint;

        if (calculatedConfigPower > MAX_POWER)
        {
            configurationError = CONFIG_POWER_LIMIT;
            return false;
        }
    }

    return true;
}

// ============================================================
// TEST CONTROL
// ============================================================

void startTest()
{
    testStartTime = millis();
    testDurationMs = (unsigned long)durationMinutes * 60UL * 1000UL;
    remainingMs = testDurationMs;
    testRunning = true;

    enableControl();
    logCsvHeader();
    logCsvRow();          // first row at t = 0
    lastLogTime = millis();

    screen = SCREEN_RUNNING;
    redraw();
}

void stopTest()
{
    testRunning = false;
    disableControl();
}

void updateTimer()
{
    if (!testRunning)
        return;

    unsigned long elapsed = millis() - testStartTime;

    if (elapsed >= testDurationMs)
    {
        remainingMs = 0;
        logCsvRow();
        logCsvEnd("COMPLETE");
        stopTest();
        screen = SCREEN_COMPLETE;
        redraw();
        return;
    }

    remainingMs = testDurationMs - elapsed;
}

// Ends the test when the battery reaches its cutoff (see BATTERY CUTOFFS)
void checkCutoff()
{
    static unsigned long conditionSince = 0;

    if (!testRunning)
    {
        conditionSince = 0;
        return;
    }

    bool reached = false;
    const char *reason = "";

    if (loadMode == MODE_CC && cutoffVoltage > 0.0 && measuredVoltage < cutoffVoltage)
    {
        reached = true;
        reason = "CUTOFF_VOLTAGE";
    }
    else if (loadMode == MODE_CV && cutoffCurrent > 0.0 && measuredCurrent > cutoffCurrent)
    {
        reached = true;
        reason = "CUTOFF_CURRENT";
    }

    if (!reached)
    {
        conditionSince = 0;
        return;
    }

    if (conditionSince == 0)
    {
        conditionSince = millis();
        return;
    }

    if (millis() - conditionSince >= CUTOFF_DELAY_MS)
    {
        conditionSince = 0;
        logCsvRow();
        logCsvEnd(reason);
        stopTest();
        screen = SCREEN_COMPLETE;
        redraw();
    }
}

// Works out which protected values are near their fault limits
void updateWarnings()
{
    float values[WARN_COUNT] = { measuredVoltage, measuredCurrent, measuredPower,
                                 maxIntTemp, maxExtTemp };
    float limits[WARN_COUNT] = { VOLTAGE_FAULT_LIMIT, CURRENT_FAULT_LIMIT, POWER_FAULT_LIMIT,
                                 INT_TEMP_FAULT_LIMIT, EXT_TEMP_FAULT_LIMIT };

    const float onLevel  = 100.0 - WARNING_PERCENT;
    const float offLevel = onLevel - WARNING_HYSTERESIS;

    worstWarning = -1;

    for (int i = 0; i < WARN_COUNT; i++)
    {
        if (isnan(values[i]) || limits[i] <= 0.0)
        {
            warnActive[i] = false;
            warnPercent[i] = 0.0;
            continue;
        }

        float pct = values[i] / limits[i] * 100.0;
        warnPercent[i] = pct;

        if (!warnActive[i] && pct >= onLevel)
        {
            warnActive[i] = true;
            Info.printf("WARNING: %s at %.0f%% of its fault limit\n", WARN_NAMES[i], pct);
        }
        else if (warnActive[i] && pct < offLevel)
        {
            warnActive[i] = false;
        }

        if (warnActive[i] && (worstWarning < 0 || pct > warnPercent[worstWarning]))
            worstWarning = i;
    }
}

// Normal header for the screens that can show the warning banner
static void drawNormalHeader()
{
    if (screen == SCREEN_RUNNING)
    {
        tft.fillRect(0, 0, 480, 45, TFT_BLUE);
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.drawString("DC ELECTRONIC LOAD - RUNNING", 15, 13);
    }
    else
    {
        drawHeader("DC ELECTRONIC LOAD");
    }
}

// Orange banner over the header while any warning is active.
// Only redrawn when its text changes, so it doesn't flash.
// force = the screen was just redrawn (header is back to normal).
void updateWarningBanner(bool force)
{
    if (screen != SCREEN_MAIN && screen != SCREEN_RUNNING)
        return;

    static char shown[48] = "";
    char text[48] = "";

    if (worstWarning >= 0)
    {
        int others = -1;
        for (int i = 0; i < WARN_COUNT; i++)
            if (warnActive[i]) others++;

        if (others > 0)
            sprintf(text, "NEAR LIMIT: %s %.0f%% (+%d)", WARN_NAMES[worstWarning],
                    warnPercent[worstWarning], others);
        else
            sprintf(text, "NEAR LIMIT: %s %.0f%%", WARN_NAMES[worstWarning],
                    warnPercent[worstWarning]);
    }

    if (force)
        shown[0] = '\0';                     // header was just drawn normally
    else if (strcmp(text, shown) == 0)
        return;                              // nothing changed

    int headerH = (screen == SCREEN_RUNNING) ? 45 : 50;
    int textY   = (screen == SCREEN_RUNNING) ? 13 : 15;

    if (text[0] == '\0')
    {
        if (shown[0] != '\0') drawNormalHeader();   // warning just cleared
        shown[0] = '\0';
        return;
    }

    if (shown[0] == '\0')
        tft.fillRect(0, 0, 480, headerH, TFT_ORANGE); // banner just appeared

    tft.setTextSize(2);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
    tft.setTextPadding(450);                  // overwrite the old text only
    tft.drawString(text, 15, textY);
    tft.setTextPadding(0);

    strncpy(shown, text, sizeof(shown) - 1);
    shown[sizeof(shown) - 1] = '\0';
}

// ============================================================
// CONTROL OUTPUT
//
// CC: setpoint / 30 A    CV: setpoint / 30 V
// Final scaling must match the PCB's analogue reference circuit.
// ============================================================

void enableControl()
{
    updateControlSignal();
}

void disableControl()
{
    analogWrite(CONTROL_PIN, 0);
}

void updateControlSignal()
{
    if (!testRunning)
    {
        disableControl();
        return;
    }

    float commandFraction;

    if (loadMode == MODE_CC)
        commandFraction = currentSetpoint / MAX_CURRENT;
    else
        commandFraction = voltageSetpoint / MAX_VOLTAGE;   // placeholder CV mapping

    commandFraction = constrain(commandFraction, 0.0, 1.0);

    analogWrite(CONTROL_PIN, (int)round(commandFraction * 255.0));
}

// ============================================================
// MENU HANDLING
// ============================================================

void handleMenu()
{
    switch (screen)
    {
    case SCREEN_MAIN:
        if (buttonPressed(BTN_UP, lastUpPress))
        {
            menuIndex--;
            if (menuIndex < 0) menuIndex = 4;
            redraw();
        }
        else if (buttonPressed(BTN_DOWN, lastDownPress))
        {
            menuIndex++;
            if (menuIndex > 4) menuIndex = 0;
            redraw();
        }
        else if (buttonPressed(BTN_SELECT, lastSelectPress))
        {
            if (menuIndex == 0)
            {
                modeIndex = (loadMode == MODE_CC) ? 0 : 1;
                screen = SCREEN_MODE;
            }
            else if (menuIndex == 1)
            {
                screen = (loadMode == MODE_CC) ? SCREEN_CURRENT : SCREEN_VOLTAGE;
            }
            else if (menuIndex == 2)
            {
                screen = SCREEN_DURATION;
            }
            else if (menuIndex == 3)
            {
                screen = SCREEN_CUTOFF;
            }
            else
            {
                screen = SCREEN_CONFIRM;
            }
            redraw();
        }
        break;

    case SCREEN_MODE:
        if (buttonPressed(BTN_UP, lastUpPress) || buttonPressed(BTN_DOWN, lastDownPress))
        {
            modeIndex = !modeIndex;
            redraw();
        }
        else if (buttonPressed(BTN_SELECT, lastSelectPress))
        {
            loadMode = (modeIndex == 0) ? MODE_CC : MODE_CV;
            screen = SCREEN_MAIN;
            redraw();
        }
        else if (buttonPressed(BTN_BACK, lastBackPress))
        {
            screen = SCREEN_MAIN;
            redraw();
        }
        break;

    case SCREEN_CURRENT:
        if (buttonPressed(BTN_UP, lastUpPress))
        {
            currentSetpoint += currentSteps[currentStepIndex];
            if (currentSetpoint > MAX_CURRENT) currentSetpoint = MAX_CURRENT;
            redraw();
        }
        else if (buttonPressed(BTN_DOWN, lastDownPress))
        {
            currentSetpoint -= currentSteps[currentStepIndex];
            if (currentSetpoint < 0.0) currentSetpoint = 0.0;
            redraw();
        }
        else if (buttonPressed(BTN_SELECT, lastSelectPress))
        {
            currentStepIndex = (currentStepIndex + 1) % 3;
            redraw();
        }
        else if (buttonPressed(BTN_BACK, lastBackPress))
        {
            screen = SCREEN_MAIN;
            redraw();
        }
        break;

    case SCREEN_VOLTAGE:
        if (buttonPressed(BTN_UP, lastUpPress))
        {
            voltageSetpoint += voltageSteps[voltageStepIndex];
            if (voltageSetpoint > MAX_VOLTAGE) voltageSetpoint = MAX_VOLTAGE;
            redraw();
        }
        else if (buttonPressed(BTN_DOWN, lastDownPress))
        {
            voltageSetpoint -= voltageSteps[voltageStepIndex];
            if (voltageSetpoint < 0.0) voltageSetpoint = 0.0;
            redraw();
        }
        else if (buttonPressed(BTN_SELECT, lastSelectPress))
        {
            voltageStepIndex = (voltageStepIndex + 1) % 3;
            redraw();
        }
        else if (buttonPressed(BTN_BACK, lastBackPress))
        {
            screen = SCREEN_MAIN;
            redraw();
        }
        break;

    case SCREEN_DURATION:
        if (buttonPressed(BTN_UP, lastUpPress))
        {
            durationMinutes += timeSteps[timeStepIndex];
            if (durationMinutes > 999) durationMinutes = 999;
            redraw();
        }
        else if (buttonPressed(BTN_DOWN, lastDownPress))
        {
            durationMinutes -= timeSteps[timeStepIndex];
            if (durationMinutes < 1) durationMinutes = 1;
            redraw();
        }
        else if (buttonPressed(BTN_SELECT, lastSelectPress))
        {
            timeStepIndex = (timeStepIndex + 1) % 4;
            redraw();
        }
        else if (buttonPressed(BTN_BACK, lastBackPress))
        {
            screen = SCREEN_MAIN;
            redraw();
        }
        break;

    case SCREEN_CUTOFF:
    {
        // CC mode adjusts the minimum voltage, CV mode the maximum current
        bool cc = (loadMode == MODE_CC);
        float &value    = cc ? cutoffVoltage : cutoffCurrent;
        int   &stepIdx  = cc ? cutoffVoltageStepIndex : cutoffCurrentStepIndex;
        float step      = cc ? voltageSteps[stepIdx] : currentSteps[stepIdx];
        float maxValue  = cc ? MAX_VOLTAGE : MAX_CURRENT;

        if (buttonPressed(BTN_UP, lastUpPress))
        {
            value += step;
            if (value > maxValue) value = maxValue;
            redraw();
        }
        else if (buttonPressed(BTN_DOWN, lastDownPress))
        {
            value -= step;
            if (value < 0.001) value = 0.0;        // 0 = OFF
            redraw();
        }
        else if (buttonPressed(BTN_SELECT, lastSelectPress))
        {
            stepIdx = (stepIdx + 1) % 3;
            redraw();
        }
        else if (buttonPressed(BTN_BACK, lastBackPress))
        {
            screen = SCREEN_MAIN;
            redraw();
        }
        break;
    }

    case SCREEN_CONFIRM:
        if (buttonPressed(BTN_SELECT, lastSelectPress))
        {
            if (configurationValid())
            {
                startTest();
            }
            else
            {
                screen = SCREEN_INVALID;
                redraw();
            }
        }
        else if (buttonPressed(BTN_BACK, lastBackPress))
        {
            screen = SCREEN_MAIN;
            redraw();
        }
        break;

    case SCREEN_INVALID:
        if (buttonPressed(BTN_BACK, lastBackPress))
        {
            screen = SCREEN_MAIN;
            redraw();
        }
        break;

    case SCREEN_RUNNING:
        if (buttonPressed(BTN_BACK, lastBackPress))
        {
            logCsvEnd("STOPPED_BY_USER");
            stopTest();
            screen = SCREEN_SAVE_PROMPT;
            redraw();
        }
        break;

    case SCREEN_COMPLETE:
        if (buttonPressed(BTN_SELECT, lastSelectPress))
        {
            screen = savePromptPending ? SCREEN_SAVE_PROMPT : SCREEN_MAIN;
            redraw();
        }
        break;

    case SCREEN_SAVE_PROMPT:
        if (buttonPressed(BTN_SELECT, lastSelectPress))          // RIGHT = YES
        {
            while (Serial.available()) Serial.read();             // clear old input
            Serial.println("SAVE");
            saveRequestTime = millis();
            saveResult = SAVE_WAITING;
            savedFileName[0] = '\0';
            savePromptPending = false;
            screen = SCREEN_SAVING;
            redraw();
        }
        else if (buttonPressed(BTN_BACK, lastBackPress))         // LEFT = NO
        {
            Serial.println("DISCARD");
            savePromptPending = false;
            screen = SCREEN_MAIN;
            redraw();
        }
        break;

    case SCREEN_SAVING:
    {
        // Always sample the button so its press detection stays in step
        bool rightPressed = buttonPressed(BTN_SELECT, lastSelectPress);

        if (saveResult == SAVE_WAITING)
        {
            checkSaveReply();
        }
        else if (rightPressed)
        {
            screen = SCREEN_MAIN;
            redraw();
        }
        break;
    }

    case SCREEN_FAULT:
        if (buttonPressed(BTN_SELECT, lastSelectPress))
        {
            // Only clear if the fault has gone
            FaultCode stillPresent = detectFault();

            if (stillPresent == FAULT_NONE)
            {
                activeFault = FAULT_NONE;
                screen = savePromptPending ? SCREEN_SAVE_PROMPT : SCREEN_MAIN;
            }
            else
            {
                activeFault = stillPresent;
            }
            redraw();
        }
        break;
    }
}

// ============================================================
// DRAWING
// ============================================================

void drawHeader(String title)
{
    tft.fillRect(0, 0, 480, 50, TFT_BLUE);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.drawString(title, 18, 15);
}

// Main-menu info panel layout
const int PANEL_LABEL_X = 186;
const int PANEL_VALUE_X = 352;
const int PANEL_ROW_Y   = 68;
const int PANEL_ROW_H   = 26;

// "< 10.50 V" (CC minimum voltage), "> 5.00 A" (CV maximum current) or "OFF"
void formatCutoff(char *buffer)
{
    if (loadMode == MODE_CC)
    {
        if (cutoffVoltage > 0.0) sprintf(buffer, "< %.2f V", cutoffVoltage);
        else                     sprintf(buffer, "OFF");
    }
    else
    {
        if (cutoffCurrent > 0.0) sprintf(buffer, "> %.2f A", cutoffCurrent);
        else                     sprintf(buffer, "OFF");
    }
}

void drawMainScreen()
{
    tft.fillScreen(TFT_BLACK);
    drawHeader("DC ELECTRONIC LOAD");

    const char *items[5] = { "Mode", "Setpoint", "Duration", "Cutoff", "Start Test" };

    tft.setTextSize(2);

    for (int i = 0; i < 5; i++)
    {
        int y = 60 + i * 44;

        if (i == menuIndex)
        {
            tft.fillRoundRect(12, y, 152, 36, 5, TFT_BLUE);
            tft.setTextColor(TFT_WHITE, TFT_BLUE);
        }
        else
        {
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
        }

        tft.drawString(items[i], 24, y + 10);
    }

    // Information panel: labels on the left, values on the right
    tft.drawRect(176, 58, 296, 222, TFT_DARKGREY);

    const char *labels[8] = { "MODE:", "SET:", "TIME:", "CUTOFF:", "IN:",
                              "MAX INT Temp:", "MAX EXT Temp:", "FAN:" };

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    for (int i = 0; i < 8; i++)
    {
        tft.drawString(labels[i], PANEL_LABEL_X, PANEL_ROW_Y + i * PANEL_ROW_H);
    }

    // Settings (only change when you press buttons)
    char buffer[40];
    tft.setTextColor(TFT_GREEN, TFT_BLACK);

    sprintf(buffer, "%s", loadMode == MODE_CC ? "CC" : "CV");
    tft.drawString(buffer, PANEL_VALUE_X, PANEL_ROW_Y + 0 * PANEL_ROW_H);

    if (loadMode == MODE_CC)
        sprintf(buffer, "%.2f A", currentSetpoint);
    else
        sprintf(buffer, "%.2f V", voltageSetpoint);
    tft.drawString(buffer, PANEL_VALUE_X, PANEL_ROW_Y + 1 * PANEL_ROW_H);

    sprintf(buffer, "%d min", durationMinutes);
    tft.drawString(buffer, PANEL_VALUE_X, PANEL_ROW_Y + 2 * PANEL_ROW_H);

    formatCutoff(buffer);
    tft.drawString(buffer, PANEL_VALUE_X, PANEL_ROW_Y + 3 * PANEL_ROW_H);

    // Live values: voltage, temps, fan (refreshed by updateMainVoltage)
    updateMainVoltage();

    // Footer
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString("UP/DOWN = MOVE", 20, 292);
    tft.drawString("RIGHT = SELECT", 275, 292);
}

void drawModeScreen()
{
    tft.fillScreen(TFT_BLACK);
    drawHeader("SELECT OPERATING MODE");

    const char *modes[2] = { "Constant Current (CC)", "Constant Voltage (CV)" };

    tft.setTextSize(2);

    for (int i = 0; i < 2; i++)
    {
        int y = 95 + i * 75;

        if (i == modeIndex)
        {
            tft.fillRoundRect(50, y, 380, 52, 6, TFT_BLUE);
            tft.setTextColor(TFT_WHITE, TFT_BLUE);
        }
        else
        {
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
        }

        tft.drawString(modes[i], 75, y + 16);
    }

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString("UP/DOWN = CHANGE", 25, 280);
    tft.drawString("RIGHT = OK    LEFT = BACK", 235, 280);
}

// Live measured voltage in the main-menu info panel. Only this line
// is redrawn (over its own black box), so the menu doesn't flash.
// Live values in the main-menu panel. Only these values are redrawn
// (each over its own black box), so the menu doesn't flash.
void updateMainVoltage()
{
    char buffer[20];

    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextPadding(112);   // clears old digits up to the panel edge

    sprintf(buffer, "%.2f V", measuredVoltage);
    tft.drawString(buffer, PANEL_VALUE_X, PANEL_ROW_Y + 4 * PANEL_ROW_H);

    formatTemp(buffer, maxIntTemp);
    tft.drawString(buffer, PANEL_VALUE_X, PANEL_ROW_Y + 5 * PANEL_ROW_H);

    formatTemp(buffer, maxExtTemp);
    tft.drawString(buffer, PANEL_VALUE_X, PANEL_ROW_Y + 6 * PANEL_ROW_H);

    if (FAN_TACH_PIN < 0)
        sprintf(buffer, "--");
    else
        sprintf(buffer, "%.0f RPM", fanRpm);
    tft.drawString(buffer, PANEL_VALUE_X, PANEL_ROW_Y + 7 * PANEL_ROW_H);

    tft.setTextPadding(0);
}

// Shared layout for the three "adjust a value" screens
void drawAdjustScreen(const char *title, const char *valueText, const char *stepText)
{
    tft.fillScreen(TFT_BLACK);
    drawHeader(title);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(4);
    tft.drawCentreString(valueText, 240, 90, 1);

    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawCentreString(stepText, 240, 175, 1);

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString("UP/DOWN = ADJUST", 30, 255);
    tft.drawString("RIGHT = CHANGE STEP", 250, 255);
    tft.drawCentreString("LEFT = BACK", 240, 285, 1);
}

void drawCurrentScreen()
{
    char value[24], step[24];
    sprintf(value, "%.2f A", currentSetpoint);
    sprintf(step, "Step: %.2f A", currentSteps[currentStepIndex]);
    drawAdjustScreen("SET CONSTANT CURRENT", value, step);
}

void drawVoltageScreen()
{
    char value[24], step[24];
    sprintf(value, "%.2f V", voltageSetpoint);
    sprintf(step, "Step: %.2f V", voltageSteps[voltageStepIndex]);
    drawAdjustScreen("SET CONSTANT VOLTAGE", value, step);
}

void drawDurationScreen()
{
    char value[24], step[24];
    sprintf(value, "%d min", durationMinutes);
    sprintf(step, "Step: %d min", timeSteps[timeStepIndex]);
    drawAdjustScreen("SET TEST DURATION", value, step);
}

void drawCutoffScreen()
{
    char value[24], step[24];
    bool cc = (loadMode == MODE_CC);

    if (cc)
    {
        if (cutoffVoltage > 0.0) sprintf(value, "%.2f V", cutoffVoltage);
        else                     sprintf(value, "OFF");
        sprintf(step, "Step: %.2f V", voltageSteps[cutoffVoltageStepIndex]);
        drawAdjustScreen("CUTOFF VOLTAGE (MINIMUM)", value, step);
    }
    else
    {
        if (cutoffCurrent > 0.0) sprintf(value, "%.2f A", cutoffCurrent);
        else                     sprintf(value, "OFF");
        sprintf(step, "Step: %.2f A", currentSteps[cutoffCurrentStepIndex]);
        drawAdjustScreen("CUTOFF CURRENT (MAXIMUM)", value, step);
    }

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawCentreString(cc ? "Stops when battery drops below this"
                            : "Stops when current rises above this",
                         240, 210, 1);
}

void drawConfirmScreen()
{
    tft.fillScreen(TFT_BLACK);
    drawHeader("CONFIRM TEST");

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    char buffer[50];

    sprintf(buffer, "Mode: %s",
            loadMode == MODE_CC ? "Constant Current" : "Constant Voltage");
    tft.drawString(buffer, 55, 68);

    if (loadMode == MODE_CC)
        sprintf(buffer, "Setpoint: %.2f A", currentSetpoint);
    else
        sprintf(buffer, "Setpoint: %.2f V", voltageSetpoint);
    tft.drawString(buffer, 55, 103);

    sprintf(buffer, "Duration: %d min", durationMinutes);
    tft.drawString(buffer, 55, 138);

    char cut[20];
    formatCutoff(cut);
    sprintf(buffer, "Cutoff: %s", cut);
    tft.drawString(buffer, 55, 173);

    sprintf(buffer, "Input Voltage: %.2f V", measuredVoltage);
    tft.drawString(buffer, 55, 208);

    if (loadMode == MODE_CC)
    {
        sprintf(buffer, "Estimated Power: %.1f W", measuredVoltage * currentSetpoint);
        tft.drawString(buffer, 55, 243);
    }

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("RIGHT = START", 55, 285);

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString("LEFT = BACK", 325, 285);
}

void drawInvalidScreen()
{
    tft.fillScreen(TFT_BLACK);
    tft.fillRect(0, 0, 480, 60, TFT_RED);

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawCentreString("INVALID CONFIGURATION", 240, 20, 1);

    const char *message;
    switch (configurationError)
    {
    case CONFIG_POWER_LIMIT:   message = "POWER LIMIT EXCEEDED";   break;
    case CONFIG_CURRENT_LIMIT: message = "CURRENT LIMIT EXCEEDED"; break;
    case CONFIG_VOLTAGE_LIMIT: message = "VOLTAGE LIMIT EXCEEDED"; break;
    case CONFIG_BELOW_CUTOFF:  message = "INPUT BELOW CUTOFF VOLTAGE"; break;
    default:                   message = "INVALID CONFIGURATION";  break;
    }

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawCentreString(message, 240, 100, 1);

    if (configurationError == CONFIG_POWER_LIMIT)
    {
        char buffer[50];
        tft.setTextColor(TFT_WHITE, TFT_BLACK);

        sprintf(buffer, "Requested: %.1f W", calculatedConfigPower);
        tft.drawCentreString(buffer, 240, 155, 1);

        sprintf(buffer, "Maximum: %.0f W", MAX_POWER);
        tft.drawCentreString(buffer, 240, 195, 1);
    }

    if (configurationError == CONFIG_BELOW_CUTOFF)
    {
        char buffer[50];
        tft.setTextColor(TFT_WHITE, TFT_BLACK);

        sprintf(buffer, "Input: %.2f V", measuredVoltage);
        tft.drawCentreString(buffer, 240, 155, 1);

        sprintf(buffer, "Cutoff: %.2f V", cutoffVoltage);
        tft.drawCentreString(buffer, 240, 195, 1);
    }

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawCentreString("LEFT = RETURN", 240, 280, 1);
}

// Static parts of the running screen: drawn ONCE when the test starts.
// The live numbers are drawn by updateRunningValues().
void drawRunningScreen()
{
    tft.fillScreen(TFT_BLACK);

    // Header
    tft.fillRect(0, 0, 480, 45, TFT_BLUE);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.drawString("DC ELECTRONIC LOAD - RUNNING", 15, 13);

    char buffer[50];

    // Mode and setpoint
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    sprintf(buffer, "MODE: %s", loadMode == MODE_CC ? "CC" : "CV");
    tft.drawString(buffer, 20, 60);

    if (loadMode == MODE_CC)
        sprintf(buffer, "SETPOINT: %.2f A", currentSetpoint);
    else
        sprintf(buffer, "SETPOINT: %.2f V", voltageSetpoint);
    tft.drawString(buffer, 250, 60);

    char cut[20];
    formatCutoff(cut);
    sprintf(buffer, "CUTOFF: %s", cut);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString(buffer, 250, 84);

    // Labels
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("VOLTAGE", 30, 110);
    tft.drawString("CURRENT", 180, 110);
    tft.drawString("POWER", 350, 110);

    tft.drawString("MAX INT Temp", 20, 205);
    tft.drawString("MAX EXT Temp", 175, 205);
    tft.drawString("TIME LEFT", 335, 205);
    tft.drawString("FAN:", 20, 268);

    // Stop hint
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString("LEFT = STOP TEST", 240, 292, 1);

    // First set of numbers
    updateRunningValues();
}

// Live numbers only. Each value is drawn over its own black box
// (text padding), so old digits are erased without clearing the
// whole screen - no flashing.
void updateRunningValues()
{
    char buffer[24];

    tft.setTextSize(2);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);

    sprintf(buffer, "%.2f V", measuredVoltage);
    tft.setTextPadding(140);
    tft.drawString(buffer, 30, 145);

    sprintf(buffer, "%.2f A", measuredCurrent);
    tft.setTextPadding(160);
    tft.drawString(buffer, 180, 145);

    sprintf(buffer, "%.1f W", measuredPower);
    tft.setTextPadding(125);
    tft.drawString(buffer, 350, 145);

    // Temperatures
    formatTemp(buffer, maxIntTemp);
    tft.setTextPadding(150);
    tft.drawString(buffer, 20, 240);

    formatTemp(buffer, maxExtTemp);
    tft.setTextPadding(150);
    tft.drawString(buffer, 175, 240);

    // Fan speed (blue wire)
    if (FAN_TACH_PIN < 0)
        sprintf(buffer, "--");
    else
        sprintf(buffer, "%.0f RPM", fanRpm);
    tft.setTextPadding(150);
    tft.drawString(buffer, 72, 268);

    // Countdown
    unsigned long totalSeconds = remainingMs / 1000UL;
    unsigned long hours = totalSeconds / 3600UL;
    unsigned int minutes = (totalSeconds % 3600UL) / 60UL;
    unsigned int seconds = totalSeconds % 60UL;

    if (hours > 0)
        sprintf(buffer, "%02lu:%02u:%02u", hours, minutes, seconds);
    else
        sprintf(buffer, "%02u:%02u", minutes, seconds);

    tft.setTextSize(3);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextPadding(144);
    tft.drawString(buffer, 335, 235);

    // Reset so other screens aren't affected
    tft.setTextPadding(0);
    tft.setTextSize(2);
}

// Human-readable reason the test ended (complete and save-prompt screens)
static const char *endReasonText()
{
    if (strcmp(lastEndReason, "COMPLETE") == 0)        return "Timer finished";
    if (strcmp(lastEndReason, "CUTOFF_VOLTAGE") == 0)  return "Cutoff voltage reached";
    if (strcmp(lastEndReason, "CUTOFF_CURRENT") == 0)  return "Cutoff current reached";
    if (strcmp(lastEndReason, "STOPPED_BY_USER") == 0) return "Test stopped by user";
    if (strncmp(lastEndReason, "FAULT", 5) == 0)       return "Test ended by a fault";
    return "Test ended";
}

void drawCompleteScreen()
{
    tft.fillScreen(TFT_BLACK);
    drawHeader("TEST COMPLETE");

    tft.setTextSize(3);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawCentreString("TEST FINISHED", 240, 95, 1);

    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawCentreString(endReasonText(), 240, 145, 1);

    char buffer[40];
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    sprintf(buffer, "Final: %.2f V  %.2f A", measuredVoltage, measuredCurrent);
    tft.drawCentreString(buffer, 240, 180, 1);
    tft.drawCentreString("Load output has been disabled", 240, 215, 1);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawCentreString("RIGHT = CONTINUE", 240, 280, 1);
}

void drawFaultScreen()
{
    tft.fillScreen(TFT_BLACK);
    tft.fillRect(0, 0, 480, 65, TFT_RED);

    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawCentreString("FAULT", 240, 18, 1);

    const char *message;
    switch (activeFault)
    {
    case FAULT_OVERVOLTAGE: message = "OVER VOLTAGE";             break;
    case FAULT_OVERCURRENT: message = "OVER CURRENT";             break;
    case FAULT_OVERPOWER:   message = "POWER LIMIT EXCEEDED";     break;
    case FAULT_OVERTEMP:     message = "INTERNAL OVER TEMPERATURE"; break;
    case FAULT_BATTERY_TEMP: message = "BATTERY OVER TEMPERATURE";  break;
    case FAULT_SENSOR:       message = "THERMISTOR OPEN/SHORT";     break;
    default:                 message = "UNKNOWN FAULT";             break;
    }

    char line[48];
    if ((activeFault == FAULT_OVERTEMP || activeFault == FAULT_BATTERY_TEMP ||
         activeFault == FAULT_SENSOR) && faultChannel >= 0)
    {
        sprintf(line, "%s (%s)", message, THERMISTORS[faultChannel].name);
        message = line;
    }

    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawCentreString(message, 240, 90, 1);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString("LOAD DISABLED", 240, 135, 1);

    char buffer[60];

    sprintf(buffer, "Voltage: %.2f V", measuredVoltage);
    tft.drawString(buffer, 60, 180);

    sprintf(buffer, "Current: %.2f A", measuredCurrent);
    tft.drawString(buffer, 270, 180);

    sprintf(buffer, "Power: %.1f W", measuredPower);
    tft.drawString(buffer, 60, 220);

    char temp[16];
    formatTemp(temp, maxIntTemp);
    sprintf(buffer, "Max INT: %s", temp);
    tft.drawString(buffer, 60, 250);

    formatTemp(temp, maxExtTemp);
    sprintf(buffer, "Max EXT: %s", temp);
    tft.drawString(buffer, 270, 250);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawCentreString("RIGHT = ACKNOWLEDGE", 240, 285, 1);
}

void drawSavePromptScreen()
{
    tft.fillScreen(TFT_BLACK);
    drawHeader("SAVE TEST LOG");

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString(endReasonText(), 240, 80, 1);

    tft.setTextSize(3);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawCentreString("SAVE LOG TO CSV?", 240, 130, 1);

    tft.setTextSize(2);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawCentreString("Laptop must be running log_capture.py", 240, 185, 1);

    // NO on the left (LEFT button), YES on the right (RIGHT button)
    tft.fillRoundRect(40, 225, 170, 55, 8, TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawCentreString("LEFT = NO", 125, 244, 1);

    tft.fillRoundRect(270, 225, 170, 55, 8, TFT_DARKGREEN);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    tft.drawCentreString("RIGHT = YES", 355, 244, 1);
}

void drawSavingScreen()
{
    tft.fillScreen(TFT_BLACK);
    drawHeader("SAVE TEST LOG");
    tft.setTextSize(2);

    if (saveResult == SAVE_WAITING)
    {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawCentreString("Saving to laptop...", 240, 140, 1);
        return;
    }

    if (saveResult == SAVE_OK)
    {
        tft.setTextSize(3);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawCentreString("LOG SAVED", 240, 100, 1);
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawCentreString(savedFileName, 240, 160, 1);
    }
    else
    {
        tft.setTextSize(3);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawCentreString("NO REPLY FROM PC", 240, 100, 1);
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawCentreString("Start log_capture.py on the laptop", 240, 160, 1);
        tft.drawCentreString("before running a test", 240, 190, 1);
    }

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawCentreString("RIGHT = RETURN TO MENU", 240, 280, 1);
}

void redraw()
{
    switch (screen)
    {
    case SCREEN_MAIN:     drawMainScreen();     break;
    case SCREEN_MODE:     drawModeScreen();     break;
    case SCREEN_CURRENT:  drawCurrentScreen();  break;
    case SCREEN_VOLTAGE:  drawVoltageScreen();  break;
    case SCREEN_DURATION: drawDurationScreen(); break;
    case SCREEN_CONFIRM:  drawConfirmScreen();  break;
    case SCREEN_INVALID:  drawInvalidScreen();  break;
    case SCREEN_RUNNING:  drawRunningScreen();  break;
    case SCREEN_COMPLETE: drawCompleteScreen(); break;
    case SCREEN_FAULT:    drawFaultScreen();    break;
    case SCREEN_SAVE_PROMPT: drawSavePromptScreen(); break;
    case SCREEN_SAVING:      drawSavingScreen();     break;
    case SCREEN_CUTOFF:      drawCutoffScreen();     break;
    }

    // Main menu / running screen: put the warning banner back if active
    updateWarningBanner(true);
}
