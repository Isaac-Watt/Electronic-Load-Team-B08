#define USER_SETUP_LOADED

#define ST7796_DRIVER

#define TFT_WIDTH  320
#define TFT_HEIGHT 480

#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   5
#define TFT_DC   17
#define TFT_RST  16

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4

#define SPI_FREQUENCY 27000000

#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

TFT_eSPI tft = TFT_eSPI();

#define BTN_UP      17
#define BTN_DOWN    18
#define BTN_BACK    21
#define BTN_SELECT  14

#define VOLTAGE_ADC_PIN     2
#define CURRENT_ADC_PIN     3
#define THERMISTOR_ADC_PIN  4

#define CONTROL_PIN         1

const float MAX_VOLTAGE = 30.0;
const float MAX_CURRENT = 30.0;
const float MAX_POWER   = 500.0;

const float VOLTAGE_FAULT_LIMIT = 30.5;
const float CURRENT_FAULT_LIMIT = 30.5;
const float POWER_FAULT_LIMIT   = 510.0;

const float TEMPERATURE_FAULT_LIMIT = 78.0;

const int ADC_SAMPLES = 32;

const float VOLTAGE_SLOPE  = 4.971;
const float VOLTAGE_OFFSET = -3.963;

float currentZeroVoltage = 1.70;

const float CURRENT_SENSITIVITY = 0.033;

const float CURRENT_DEADBAND = 0.10;

const float THERMISTOR_NOMINAL_RESISTANCE = 10000.0;

const float THERMISTOR_NOMINAL_TEMPERATURE = 25.0;

const float THERMISTOR_BETA = 3950.0;

const float SERIES_RESISTOR = 10000.0;

const float ADC_SUPPLY = 3.3;

const bool THERMISTOR_REQUIRED = true;

enum LoadMode
{
    MODE_CC,
    MODE_CV
};

LoadMode loadMode = MODE_CC;

enum Screen
{
    SCREEN_MAIN,
    SCREEN_MODE,
    SCREEN_CURRENT,
    SCREEN_VOLTAGE,
    SCREEN_DURATION,
    SCREEN_CONFIRM,
    SCREEN_INVALID,
    SCREEN_RUNNING,
    SCREEN_COMPLETE,
    SCREEN_FAULT
};

Screen screen = SCREEN_MAIN;

enum FaultCode
{
    FAULT_NONE,
    FAULT_OVERVOLTAGE,
    FAULT_OVERCURRENT,
    FAULT_OVERPOWER,
    FAULT_OVERTEMP,
    FAULT_SENSOR
};

FaultCode activeFault = FAULT_NONE;

enum ConfigError
{
    CONFIG_OK,
    CONFIG_CURRENT_LIMIT,
    CONFIG_VOLTAGE_LIMIT,
    CONFIG_POWER_LIMIT
};

ConfigError configurationError = CONFIG_OK;

int menuIndex = 0;

int modeIndex = 0;

float currentSetpoint = 1.00;

float voltageSetpoint = 5.00;

int durationMinutes = 10;

float currentSteps[] =
{
    0.01,
    0.10,
    1.00
};

int currentStepIndex = 0;

float voltageSteps[] =
{
    0.10,
    1.00,
    5.00
};

int voltageStepIndex = 0;

int timeSteps[] =
{
    1,
    5,
    10,
    30
};

int timeStepIndex = 0;

float measuredVoltage = 0.0;

float measuredCurrent = 0.0;

float measuredPower = 0.0;

float measuredTemperature = 0.0;

bool testRunning = false;

unsigned long testStartTime = 0;

unsigned long testDurationMs = 0;

unsigned long remainingMs = 0;

unsigned long lastDisplayUpdate = 0;

const unsigned long DISPLAY_UPDATE_INTERVAL = 250;

const unsigned long BUTTON_DEBOUNCE = 180;

unsigned long lastUpPress = 0;
unsigned long lastDownPress = 0;
unsigned long lastBackPress = 0;
unsigned long lastSelectPress = 0;

float calculatedConfigPower = 0.0;

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

void drawCompleteScreen();

void drawFaultScreen();

void handleMenu();

bool buttonPressed(
    int pin,
    unsigned long &lastPress
);

float readADCVoltageRaw(int pin);

void readMeasurements();

float readVoltage();

float readCurrent();

float readTemperature();

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

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();

    Serial.println(
        "===================================="
    );

    Serial.println(
        "PROGRAMMABLE DC ELECTRONIC LOAD"
    );

    Serial.println(
        "===================================="
    );

    pinMode(
        BTN_UP,
        INPUT
    );

    pinMode(
        BTN_DOWN,
        INPUT
    );

    pinMode(
        BTN_BACK,
        INPUT
    );

    pinMode(
        BTN_SELECT,
        INPUT
    );

    pinMode(
        CONTROL_PIN,
        OUTPUT
    );

    disableControl();

    analogReadResolution(12);

    analogSetPinAttenuation(
        VOLTAGE_ADC_PIN,
        ADC_11db
    );

    analogSetPinAttenuation(
        CURRENT_ADC_PIN,
        ADC_11db
    );

    analogSetPinAttenuation(
        THERMISTOR_ADC_PIN,
        ADC_11db
    );

    calibrateCurrentZero();

    tft.init();

    tft.setRotation(1);

    tft.fillScreen(
        TFT_BLACK
    );

    redraw();

    Serial.println();

    Serial.println(
        "System initialised."
    );

    Serial.println();

    Serial.println(
        "Temporary pin map:"
    );

    Serial.println(
        "Voltage ADC  = GPIO2"
    );

    Serial.println(
        "Current ADC  = GPIO3"
    );

    Serial.println(
        "Thermistor   = GPIO4"
    );

    Serial.println(
        "Control Out  = GPIO1"
    );

    Serial.println();

    Serial.print(
        "ACS712 zero voltage = "
    );

    Serial.print(
        currentZeroVoltage,
        4
    );

    Serial.println(
        " V"
    );
}

void loop()
{
    readMeasurements();

    checkFaults();

    if (
        activeFault !=
        FAULT_NONE
    )
    {
        disableControl();

        testRunning = false;

        if (
            screen !=
            SCREEN_FAULT
        )
        {
            screen =
                SCREEN_FAULT;

            redraw();
        }

        handleMenu();

        return;
    }

    if (
        screen ==
        SCREEN_RUNNING
    )
    {
        updateTimer();

        if (
            screen ==
            SCREEN_RUNNING
        )
        {
            updateControlSignal();

            if (
                millis() -
                lastDisplayUpdate
                >=
                DISPLAY_UPDATE_INTERVAL
            )
            {
                lastDisplayUpdate =
                    millis();

                drawRunningScreen();
            }
        }
    }

    handleMenu();
}

float readADCVoltageRaw(int pin)
{
    uint32_t total = 0;

    for (
        int i = 0;
        i < ADC_SAMPLES;
        i++
    )
    {
        total +=
            analogRead(pin);
    }

    float average =
        total /
        (float)ADC_SAMPLES;

    float voltage =
        (
            average /
            4095.0
        )
        *
        3.3;

    return voltage;
}

void calibrateCurrentZero()
{
    Serial.println();

    Serial.println(
        "Calibrating current sensor zero..."
    );

    Serial.println(
        "Ensure NO current is flowing."
    );

    delay(500);

    const int zeroSamples = 200;

    float total = 0.0;

    for (
        int i = 0;
        i < zeroSamples;
        i++
    )
    {
        total +=
            readADCVoltageRaw(
                CURRENT_ADC_PIN
            );

        delay(3);
    }

    currentZeroVoltage =
        total /
        zeroSamples;

    Serial.print(
        "Current sensor zero = "
    );

    Serial.print(
        currentZeroVoltage,
        4
    );

    Serial.println(
        " V"
    );
}

bool buttonPressed(
    int pin,
    unsigned long &lastPress
)
{
    if (
        digitalRead(pin)
        ==
        LOW
    )
    {
        if (
            millis() -
            lastPress
            >
            BUTTON_DEBOUNCE
        )
        {
            lastPress =
                millis();

            return true;
        }
    }

    return false;
}

void readMeasurements()
{
    measuredVoltage =
        readVoltage();

    measuredCurrent =
        readCurrent();

    measuredTemperature =
        readTemperature();

    measuredPower =
        measuredVoltage
        *
        measuredCurrent;
}

float readVoltage()
{
    float adcVoltage =
        readADCVoltageRaw(
            VOLTAGE_ADC_PIN
        );

    float actualVoltage =
        (
            VOLTAGE_SLOPE
            *
            adcVoltage
        )
        +
        VOLTAGE_OFFSET;

    if (
        actualVoltage <
        0.0
    )
    {
        actualVoltage =
            0.0;
    }

    return actualVoltage;
}

float readCurrent()
{
    float adcVoltage =
        readADCVoltageRaw(
            CURRENT_ADC_PIN
        );

    float current =
        (
            adcVoltage -
            currentZeroVoltage
        )
        /
        CURRENT_SENSITIVITY;

    current =
        fabs(current);

    if (
        current <
        CURRENT_DEADBAND
    )
    {
        current =
            0.0;
    }

    return current;
}

float readTemperature()
{
    int millivolts =
        analogReadMilliVolts(
            THERMISTOR_ADC_PIN
        );

    float voltage =
        millivolts /
        1000.0;

    if (
        voltage <=
        0.01
        ||
        voltage >=
        ADC_SUPPLY -
        0.01
    )
    {
        return NAN;
    }

    float resistance =
        SERIES_RESISTOR
        *
        voltage
        /
        (
            ADC_SUPPLY -
            voltage
        );

    float T0 =
        THERMISTOR_NOMINAL_TEMPERATURE
        +
        273.15;

    float temperatureKelvin =
        1.0
        /
        (
            (1.0 / T0)
            +
            (1.0 /
            THERMISTOR_BETA)
            *
            log(
                resistance /
                THERMISTOR_NOMINAL_RESISTANCE
            )
        );

    return
        temperatureKelvin -
        273.15;
}

FaultCode detectFault()
{
    if (
        THERMISTOR_REQUIRED
        &&
        isnan(
            measuredTemperature
        )
    )
    {
        return
            FAULT_SENSOR;
    }

    if (
        !isnan(
            measuredTemperature
        )
        &&
        measuredTemperature >
        TEMPERATURE_FAULT_LIMIT
    )
    {
        return
            FAULT_OVERTEMP;
    }

    if (
        measuredVoltage >
        VOLTAGE_FAULT_LIMIT
    )
    {
        return
            FAULT_OVERVOLTAGE;
    }

    if (
        measuredCurrent >
        CURRENT_FAULT_LIMIT
    )
    {
        return
            FAULT_OVERCURRENT;
    }

    if (
        measuredPower >
        POWER_FAULT_LIMIT
    )
    {
        return
            FAULT_OVERPOWER;
    }

    return
        FAULT_NONE;
}

void checkFaults()
{
    if (
        activeFault !=
        FAULT_NONE
    )
    {
        return;
    }

    FaultCode detected =
        detectFault();

    if (
        detected !=
        FAULT_NONE
    )
    {
        activeFault =
            detected;

        disableControl();

        testRunning =
            false;
    }
}

bool configurationValid()
{
    configurationError =
        CONFIG_OK;

    calculatedConfigPower =
        0.0;

    if (
        currentSetpoint >
        MAX_CURRENT
    )
    {
        configurationError =
            CONFIG_CURRENT_LIMIT;

        return false;
    }

    if (
        voltageSetpoint >
        MAX_VOLTAGE
    )
    {
        configurationError =
            CONFIG_VOLTAGE_LIMIT;

        return false;
    }

    if (
        loadMode ==
        MODE_CC
    )
    {
        calculatedConfigPower =
            measuredVoltage
            *
            currentSetpoint;

        if (
            calculatedConfigPower >
            MAX_POWER
        )
        {
            configurationError =
                CONFIG_POWER_LIMIT;

            return false;
        }
    }

    return true;
}

void startTest()
{
    testStartTime =
        millis();

    testDurationMs =
        (
            unsigned long
        )
        durationMinutes
        *
        60UL
        *
        1000UL;

    remainingMs =
        testDurationMs;

    testRunning =
        true;

    enableControl();

    screen =
        SCREEN_RUNNING;

    redraw();
}

void stopTest()
{
    testRunning =
        false;

    disableControl();
}

void updateTimer()
{
    if (
        !testRunning
    )
    {
        return;
    }

    unsigned long elapsed =
        millis()
        -
        testStartTime;

    if (
        elapsed >=
        testDurationMs
    )
    {
        remainingMs = 0;

        stopTest();

        screen =
            SCREEN_COMPLETE;

        redraw();

        return;
    }

    remainingMs =
        testDurationMs
        -
        elapsed;
}

void enableControl()
{
    updateControlSignal();
}

void disableControl()
{
    analogWrite(
        CONTROL_PIN,
        0
    );
}

void updateControlSignal()
{
    if (
        !testRunning
    )
    {
        disableControl();

        return;
    }

    float commandFraction =
        0.0;

    if (
        loadMode ==
        MODE_CC
    )
    {
        commandFraction =
            currentSetpoint /
            MAX_CURRENT;
    }

    else
    {
        commandFraction =
            voltageSetpoint /
            MAX_VOLTAGE;
    }

    commandFraction =
        constrain(
            commandFraction,
            0.0,
            1.0
        );

    int pwmValue =
        round(
            commandFraction
            *
            255.0
        );

    analogWrite(
        CONTROL_PIN,
        pwmValue
    );
}

void handleMenu()
{
    if (
        screen ==
        SCREEN_MAIN
    )
    {
        if (
            buttonPressed(
                BTN_UP,
                lastUpPress
            )
        )
        {
            menuIndex--;

            if (
                menuIndex <
                0
            )
            {
                menuIndex =
                    3;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_DOWN,
                lastDownPress
            )
        )
        {
            menuIndex++;

            if (
                menuIndex >
                3
            )
            {
                menuIndex =
                    0;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_SELECT,
                lastSelectPress
            )
        )
        {
            if (
                menuIndex ==
                0
            )
            {
                modeIndex =
                    (
                        loadMode ==
                        MODE_CC
                    )
                    ?
                    0
                    :
                    1;

                screen =
                    SCREEN_MODE;
            }

            else if (
                menuIndex ==
                1
            )
            {
                if (
                    loadMode ==
                    MODE_CC
                )
                {
                    screen =
                        SCREEN_CURRENT;
                }

                else
                {
                    screen =
                        SCREEN_VOLTAGE;
                }
            }

            else if (
                menuIndex ==
                2
            )
            {
                screen =
                    SCREEN_DURATION;
            }

            else if (
                menuIndex ==
                3
            )
            {
                screen =
                    SCREEN_CONFIRM;
            }

            redraw();
        }
    }

    else if (
        screen ==
        SCREEN_MODE
    )
    {
        if (
            buttonPressed(
                BTN_UP,
                lastUpPress
            )
            ||
            buttonPressed(
                BTN_DOWN,
                lastDownPress
            )
        )
        {
            modeIndex =
                !modeIndex;

            redraw();
        }

        else if (
            buttonPressed(
                BTN_SELECT,
                lastSelectPress
            )
        )
        {
            if (
                modeIndex ==
                0
            )
            {
                loadMode =
                    MODE_CC;
            }

            else
            {
                loadMode =
                    MODE_CV;
            }

            screen =
                SCREEN_MAIN;

            redraw();
        }

        else if (
            buttonPressed(
                BTN_BACK,
                lastBackPress
            )
        )
        {
            screen =
                SCREEN_MAIN;

            redraw();
        }
    }

    else if (
        screen ==
        SCREEN_CURRENT
    )
    {
        if (
            buttonPressed(
                BTN_UP,
                lastUpPress
            )
        )
        {
            currentSetpoint +=
                currentSteps[
                    currentStepIndex
                ];

            if (
                currentSetpoint >
                MAX_CURRENT
            )
            {
                currentSetpoint =
                    MAX_CURRENT;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_DOWN,
                lastDownPress
            )
        )
        {
            currentSetpoint -=
                currentSteps[
                    currentStepIndex
                ];

            if (
                currentSetpoint <
                0.0
            )
            {
                currentSetpoint =
                    0.0;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_SELECT,
                lastSelectPress
            )
        )
        {
            currentStepIndex++;

            if (
                currentStepIndex >
                2
            )
            {
                currentStepIndex =
                    0;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_BACK,
                lastBackPress
            )
        )
        {
            screen =
                SCREEN_MAIN;

            redraw();
        }
    }

    else if (
        screen ==
        SCREEN_VOLTAGE
    )
    {
        if (
            buttonPressed(
                BTN_UP,
                lastUpPress
            )
        )
        {
            voltageSetpoint +=
                voltageSteps[
                    voltageStepIndex
                ];

            if (
                voltageSetpoint >
                MAX_VOLTAGE
            )
            {
                voltageSetpoint =
                    MAX_VOLTAGE;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_DOWN,
                lastDownPress
            )
        )
        {
            voltageSetpoint -=
                voltageSteps[
                    voltageStepIndex
                ];

            if (
                voltageSetpoint <
                0.0
            )
            {
                voltageSetpoint =
                    0.0;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_SELECT,
                lastSelectPress
            )
        )
        {
            voltageStepIndex++;

            if (
                voltageStepIndex >
                2
            )
            {
                voltageStepIndex =
                    0;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_BACK,
                lastBackPress
            )
        )
        {
            screen =
                SCREEN_MAIN;

            redraw();
        }
    }

    else if (
        screen ==
        SCREEN_DURATION
    )
    {
        if (
            buttonPressed(
                BTN_UP,
                lastUpPress
            )
        )
        {
            durationMinutes +=
                timeSteps[
                    timeStepIndex
                ];

            if (
                durationMinutes >
                999
            )
            {
                durationMinutes =
                    999;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_DOWN,
                lastDownPress
            )
        )
        {
            durationMinutes -=
                timeSteps[
                    timeStepIndex
                ];

            if (
                durationMinutes <
                1
            )
            {
                durationMinutes =
                    1;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_SELECT,
                lastSelectPress
            )
        )
        {
            timeStepIndex++;

            if (
                timeStepIndex >
                3
            )
            {
                timeStepIndex =
                    0;
            }

            redraw();
        }

        else if (
            buttonPressed(
                BTN_BACK,
                lastBackPress
            )
        )
        {
            screen =
                SCREEN_MAIN;

            redraw();
        }
    }

    else if (
        screen ==
        SCREEN_CONFIRM
    )
    {
        if (
            buttonPressed(
                BTN_SELECT,
                lastSelectPress
            )
        )
        {
            if (
                configurationValid()
            )
            {
                startTest();
            }

            else
            {
                screen =
                    SCREEN_INVALID;

                redraw();
            }
        }

        else if (
            buttonPressed(
                BTN_BACK,
                lastBackPress
            )
        )
        {
            screen =
                SCREEN_MAIN;

            redraw();
        }
    }

    else if (
        screen ==
        SCREEN_INVALID
    )
    {
        if (
            buttonPressed(
                BTN_BACK,
                lastBackPress
            )
        )
        {
            screen =
                SCREEN_MAIN;

            redraw();
        }
    }

    else if (
        screen ==
        SCREEN_RUNNING
    )
    {
        if (
            buttonPressed(
                BTN_BACK,
                lastBackPress
            )
        )
        {
            stopTest();

            screen =
                SCREEN_MAIN;

            redraw();
        }
    }

    else if (
        screen ==
        SCREEN_COMPLETE
    )
    {
        if (
            buttonPressed(
                BTN_SELECT,
                lastSelectPress
            )
        )
        {
            screen =
                SCREEN_MAIN;

            redraw();
        }
    }

    else if (
        screen ==
        SCREEN_FAULT
    )
    {
        if (
            buttonPressed(
                BTN_SELECT,
                lastSelectPress
            )
        )
        {
            FaultCode stillPresent =
                detectFault();

            if (
                stillPresent ==
                FAULT_NONE
            )
            {
                activeFault =
                    FAULT_NONE;

                screen =
                    SCREEN_MAIN;

                redraw();
            }

            else
            {
                activeFault =
                    stillPresent;

                redraw();
            }
        }
    }
}

void drawHeader(
    String title
)
{
    tft.fillRect(
        0,
        0,
        480,
        50,
        TFT_BLUE
    );

    tft.setTextSize(2);

    tft.setTextColor(
        TFT_WHITE,
        TFT_BLUE
    );

    tft.drawString(
        title,
        18,
        15
    );
}

void drawMainScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    drawHeader(
        "DC ELECTRONIC LOAD"
    );

    const char*
    items[4] =
    {
        "Mode",
        "Setpoint",
        "Duration",
        "Start Test"
    };

    tft.setTextSize(2);

    for (
        int i = 0;
        i < 4;
        i++
    )
    {
        int y =
            68 +
            i * 46;

        if (
            i ==
            menuIndex
        )
        {
            tft.fillRoundRect(
                20,
                y,
                270,
                38,
                5,
                TFT_BLUE
            );

            tft.setTextColor(
                TFT_WHITE,
                TFT_BLUE
            );
        }

        else
        {
            tft.setTextColor(
                TFT_WHITE,
                TFT_BLACK
            );
        }

        tft.drawString(
            items[i],
            38,
            y + 9
        );
    }

    tft.drawRect(
        310,
        70,
        155,
        180,
        TFT_DARKGREY
    );

    tft.setTextColor(
        TFT_GREEN,
        TFT_BLACK
    );

    char buffer[40];

    sprintf(
        buffer,
        "MODE: %s",
        loadMode ==
        MODE_CC
        ?
        "CC"
        :
        "CV"
    );

    tft.drawString(
        buffer,
        325,
        90
    );

    if (
        loadMode ==
        MODE_CC
    )
    {
        sprintf(
            buffer,
            "SET: %.2f A",
            currentSetpoint
        );
    }

    else
    {
        sprintf(
            buffer,
            "SET: %.2f V",
            voltageSetpoint
        );
    }

    tft.drawString(
        buffer,
        325,
        130
    );

    sprintf(
        buffer,
        "TIME: %d min",
        durationMinutes
    );

    tft.drawString(
        buffer,
        325,
        170
    );

    tft.setTextColor(
        TFT_LIGHTGREY,
        TFT_BLACK
    );

    tft.drawString(
        "UP/DOWN = MOVE",
        20,
        285
    );

    tft.drawString(
        "RIGHT = SELECT",
        275,
        285
    );
}

void drawModeScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    drawHeader(
        "SELECT OPERATING MODE"
    );

    const char*
    modes[2] =
    {
        "Constant Current (CC)",
        "Constant Voltage (CV)"
    };

    tft.setTextSize(2);

    for (
        int i = 0;
        i < 2;
        i++
    )
    {
        int y =
            95 +
            i * 75;

        if (
            i ==
            modeIndex
        )
        {
            tft.fillRoundRect(
                50,
                y,
                380,
                52,
                6,
                TFT_BLUE
            );

            tft.setTextColor(
                TFT_WHITE,
                TFT_BLUE
            );
        }

        else
        {
            tft.setTextColor(
                TFT_WHITE,
                TFT_BLACK
            );
        }

        tft.drawString(
            modes[i],
            75,
            y + 16
        );
    }

    tft.setTextColor(
        TFT_LIGHTGREY,
        TFT_BLACK
    );

    tft.drawString(
        "UP/DOWN = CHANGE",
        25,
        280
    );

    tft.drawString(
        "RIGHT = OK    LEFT = BACK",
        235,
        280
    );
}

void drawCurrentScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    drawHeader(
        "SET CONSTANT CURRENT"
    );

    char buffer[40];

    sprintf(
        buffer,
        "%.2f A",
        currentSetpoint
    );

    tft.setTextColor(
        TFT_GREEN,
        TFT_BLACK
    );

    tft.setTextSize(4);

    tft.drawCentreString(
        buffer,
        240,
        90,
        1
    );

    tft.setTextSize(2);

    sprintf(
        buffer,
        "Step: %.2f A",
        currentSteps[
            currentStepIndex
        ]
    );

    tft.setTextColor(
        TFT_YELLOW,
        TFT_BLACK
    );

    tft.drawCentreString(
        buffer,
        240,
        175,
        1
    );

    tft.setTextColor(
        TFT_LIGHTGREY,
        TFT_BLACK
    );

    tft.drawString(
        "UP/DOWN = ADJUST",
        30,
        255
    );

    tft.drawString(
        "RIGHT = CHANGE STEP",
        270,
        255
    );

    tft.drawCentreString(
        "LEFT = BACK",
        240,
        285,
        1
    );
}

void drawVoltageScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    drawHeader(
        "SET CONSTANT VOLTAGE"
    );

    char buffer[40];

    sprintf(
        buffer,
        "%.2f V",
        voltageSetpoint
    );

    tft.setTextColor(
        TFT_GREEN,
        TFT_BLACK
    );

    tft.setTextSize(4);

    tft.drawCentreString(
        buffer,
        240,
        90,
        1
    );

    tft.setTextSize(2);

    sprintf(
        buffer,
        "Step: %.2f V",
        voltageSteps[
            voltageStepIndex
        ]
    );

    tft.setTextColor(
        TFT_YELLOW,
        TFT_BLACK
    );

    tft.drawCentreString(
        buffer,
        240,
        175,
        1
    );

    tft.setTextColor(
        TFT_LIGHTGREY,
        TFT_BLACK
    );

    tft.drawString(
        "UP/DOWN = ADJUST",
        30,
        255
    );

    tft.drawString(
        "RIGHT = CHANGE STEP",
        270,
        255
    );

    tft.drawCentreString(
        "LEFT = BACK",
        240,
        285,
        1
    );
}

void drawDurationScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    drawHeader(
        "SET TEST DURATION"
    );

    char buffer[40];

    sprintf(
        buffer,
        "%d min",
        durationMinutes
    );

    tft.setTextColor(
        TFT_GREEN,
        TFT_BLACK
    );

    tft.setTextSize(4);

    tft.drawCentreString(
        buffer,
        240,
        90,
        1
    );

    tft.setTextSize(2);

    sprintf(
        buffer,
        "Step: %d min",
        timeSteps[
            timeStepIndex
        ]
    );

    tft.setTextColor(
        TFT_YELLOW,
        TFT_BLACK
    );

    tft.drawCentreString(
        buffer,
        240,
        175,
        1
    );

    tft.setTextColor(
        TFT_LIGHTGREY,
        TFT_BLACK
    );

    tft.drawString(
        "UP/DOWN = ADJUST",
        30,
        255
    );

    tft.drawString(
        "RIGHT = CHANGE STEP",
        270,
        255
    );

    tft.drawCentreString(
        "LEFT = BACK",
        240,
        285,
        1
    );
}

void drawConfirmScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    drawHeader(
        "CONFIRM TEST"
    );

    tft.setTextSize(2);

    tft.setTextColor(
        TFT_WHITE,
        TFT_BLACK
    );

    char buffer[50];

    sprintf(
        buffer,
        "Mode: %s",
        loadMode ==
        MODE_CC
        ?
        "Constant Current"
        :
        "Constant Voltage"
    );

    tft.drawString(
        buffer,
        55,
        75
    );

    if (
        loadMode ==
        MODE_CC
    )
    {
        sprintf(
            buffer,
            "Setpoint: %.2f A",
            currentSetpoint
        );
    }

    else
    {
        sprintf(
            buffer,
            "Setpoint: %.2f V",
            voltageSetpoint
        );
    }

    tft.drawString(
        buffer,
        55,
        115
    );

    sprintf(
        buffer,
        "Duration: %d min",
        durationMinutes
    );

    tft.drawString(
        buffer,
        55,
        155
    );

    sprintf(
        buffer,
        "Input Voltage: %.2f V",
        measuredVoltage
    );

    tft.drawString(
        buffer,
        55,
        195
    );

    if (
        loadMode ==
        MODE_CC
    )
    {
        float expectedPower =
            measuredVoltage
            *
            currentSetpoint;

        sprintf(
            buffer,
            "Estimated Power: %.1f W",
            expectedPower
        );

        tft.drawString(
            buffer,
            55,
            235
        );
    }

    tft.setTextColor(
        TFT_GREEN,
        TFT_BLACK
    );

    tft.drawString(
        "RIGHT = START",
        55,
        285
    );

    tft.setTextColor(
        TFT_LIGHTGREY,
        TFT_BLACK
    );

    tft.drawString(
        "LEFT = BACK",
        325,
        285
    );
}

void drawInvalidScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    tft.fillRect(
        0,
        0,
        480,
        60,
        TFT_RED
    );

    tft.setTextSize(2);

    tft.setTextColor(
        TFT_WHITE,
        TFT_RED
    );

    tft.drawCentreString(
        "INVALID CONFIGURATION",
        240,
        20,
        1
    );

    tft.setTextColor(
        TFT_RED,
        TFT_BLACK
    );

    String message;

    if (
        configurationError ==
        CONFIG_POWER_LIMIT
    )
    {
        message =
            "POWER LIMIT EXCEEDED";
    }

    else if (
        configurationError ==
        CONFIG_CURRENT_LIMIT
    )
    {
        message =
            "CURRENT LIMIT EXCEEDED";
    }

    else if (
        configurationError ==
        CONFIG_VOLTAGE_LIMIT
    )
    {
        message =
            "VOLTAGE LIMIT EXCEEDED";
    }

    else
    {
        message =
            "INVALID CONFIGURATION";
    }

    tft.drawCentreString(
        message,
        240,
        100,
        1
    );

    char buffer[50];

    if (
        configurationError ==
        CONFIG_POWER_LIMIT
    )
    {
        tft.setTextColor(
            TFT_WHITE,
            TFT_BLACK
        );

        sprintf(
            buffer,
            "Requested: %.1f W",
            calculatedConfigPower
        );

        tft.drawCentreString(
            buffer,
            240,
            155,
            1
        );

        sprintf(
            buffer,
            "Maximum: %.0f W",
            MAX_POWER
        );

        tft.drawCentreString(
            buffer,
            240,
            195,
            1
        );
    }

    tft.setTextColor(
        TFT_YELLOW,
        TFT_BLACK
    );

    tft.drawCentreString(
        "LEFT = RETURN",
        240,
        280,
        1
    );
}

void drawRunningScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    tft.fillRect(
        0,
        0,
        480,
        45,
        TFT_BLUE
    );

    tft.setTextSize(2);

    tft.setTextColor(
        TFT_WHITE,
        TFT_BLUE
    );

    tft.drawString(
        "DC ELECTRONIC LOAD - RUNNING",
        15,
        13
    );

    char buffer[50];

    tft.setTextColor(
        TFT_WHITE,
        TFT_BLACK
    );

    sprintf(
        buffer,
        "MODE: %s",
        loadMode ==
        MODE_CC
        ?
        "CC"
        :
        "CV"
    );

    tft.drawString(
        buffer,
        20,
        60
    );

    if (
        loadMode ==
        MODE_CC
    )
    {
        sprintf(
            buffer,
            "SETPOINT: %.2f A",
            currentSetpoint
        );
    }

    else
    {
        sprintf(
            buffer,
            "SETPOINT: %.2f V",
            voltageSetpoint
        );
    }

    tft.drawString(
        buffer,
        250,
        60
    );

    tft.setTextColor(
        TFT_CYAN,
        TFT_BLACK
    );

    tft.drawString(
        "VOLTAGE",
        30,
        110
    );

    tft.drawString(
        "CURRENT",
        180,
        110
    );

    tft.drawString(
        "POWER",
        350,
        110
    );

    tft.setTextColor(
        TFT_GREEN,
        TFT_BLACK
    );

    sprintf(
        buffer,
        "%.2f V",
        measuredVoltage
    );

    tft.drawString(
        buffer,
        30,
        145
    );

    sprintf(
        buffer,
        "%.2f A",
        measuredCurrent
    );

    tft.drawString(
        buffer,
        180,
        145
    );

    sprintf(
        buffer,
        "%.1f W",
        measuredPower
    );

    tft.drawString(
        buffer,
        350,
        145
    );

    tft.setTextColor(
        TFT_CYAN,
        TFT_BLACK
    );

    tft.drawString(
        "TEMPERATURE",
        35,
        205
    );

    tft.setTextColor(
        TFT_GREEN,
        TFT_BLACK
    );

    if (
        isnan(
            measuredTemperature
        )
    )
    {
        tft.drawString(
            "SENSOR FAULT",
            35,
            240
        );
    }

    else
    {
        sprintf(
            buffer,
            "%.1f C",
            measuredTemperature
        );

        tft.drawString(
            buffer,
            35,
            240
        );
    }

    unsigned long totalSeconds =
        remainingMs /
        1000UL;

    unsigned long hours =
        totalSeconds /
        3600UL;

    unsigned int minutes =
        (
            totalSeconds %
            3600UL
        )
        /
        60UL;

    unsigned int seconds =
        totalSeconds %
        60UL;

    tft.setTextColor(
        TFT_CYAN,
        TFT_BLACK
    );

    tft.drawString(
        "TIME LEFT",
        300,
        205
    );

    if (
        hours >
        0
    )
    {
        sprintf(
            buffer,
            "%02lu:%02u:%02u",
            hours,
            minutes,
            seconds
        );
    }

    else
    {
        sprintf(
            buffer,
            "%02u:%02u",
            minutes,
            seconds
        );
    }

    tft.setTextSize(3);

    tft.setTextColor(
        TFT_YELLOW,
        TFT_BLACK
    );

    tft.drawString(
        buffer,
        290,
        235
    );

    tft.setTextSize(2);

    tft.setTextColor(
        TFT_WHITE,
        TFT_BLACK
    );

    tft.drawCentreString(
        "LEFT = STOP TEST",
        240,
        292,
        1
    );
}

void drawCompleteScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    drawHeader(
        "TEST COMPLETE"
    );

    tft.setTextSize(3);

    tft.setTextColor(
        TFT_GREEN,
        TFT_BLACK
    );

    tft.drawCentreString(
        "TEST FINISHED",
        240,
        110,
        1
    );

    tft.setTextSize(2);

    tft.setTextColor(
        TFT_WHITE,
        TFT_BLACK
    );

    tft.drawCentreString(
        "Load output has been disabled",
        240,
        180,
        1
    );

    tft.setTextColor(
        TFT_YELLOW,
        TFT_BLACK
    );

    tft.drawCentreString(
        "RIGHT = RETURN TO MENU",
        240,
        280,
        1
    );
}

void drawFaultScreen()
{
    tft.fillScreen(
        TFT_BLACK
    );

    tft.fillRect(
        0,
        0,
        480,
        65,
        TFT_RED
    );

    tft.setTextSize(3);

    tft.setTextColor(
        TFT_WHITE,
        TFT_RED
    );

    tft.drawCentreString(
        "FAULT",
        240,
        18,
        1
    );

    String message;

    switch (
        activeFault
    )
    {
        case FAULT_OVERVOLTAGE:

            message =
                "OVER VOLTAGE";

            break;

        case FAULT_OVERCURRENT:

            message =
                "OVER CURRENT";

            break;

        case FAULT_OVERPOWER:

            message =
                "POWER LIMIT EXCEEDED";

            break;

        case FAULT_OVERTEMP:

            message =
                "OVER TEMPERATURE";

            break;

        case FAULT_SENSOR:

            message =
                "TEMPERATURE SENSOR FAULT";

            break;

        default:

            message =
                "UNKNOWN FAULT";

            break;
    }

    tft.setTextSize(2);

    tft.setTextColor(
        TFT_RED,
        TFT_BLACK
    );

    tft.drawCentreString(
        message,
        240,
        90,
        1
    );

    tft.setTextColor(
        TFT_WHITE,
        TFT_BLACK
    );

    tft.drawCentreString(
        "LOAD DISABLED",
        240,
        135,
        1
    );

    char buffer[60];

    sprintf(
        buffer,
        "Voltage: %.2f V",
        measuredVoltage
    );

    tft.drawString(
        buffer,
        60,
        180
    );

    sprintf(
        buffer,
        "Current: %.2f A",
        measuredCurrent
    );

    tft.drawString(
        buffer,
        270,
        180
    );

    sprintf(
        buffer,
        "Power: %.1f W",
        measuredPower
    );

    tft.drawString(
        buffer,
        60,
        220
    );

    if (
        isnan(
            measuredTemperature
        )
    )
    {
        sprintf(
            buffer,
            "Temp: FAULT"
        );
    }

    else
    {
        sprintf(
            buffer,
            "Temp: %.1f C",
            measuredTemperature
        );
    }

    tft.drawString(
        buffer,
        270,
        220
    );

    tft.setTextColor(
        TFT_YELLOW,
        TFT_BLACK
    );

    tft.drawCentreString(
        "RIGHT = ACKNOWLEDGE",
        240,
        285,
        1
    );
}

void redraw()
{
    switch (
        screen
    )
    {
        case SCREEN_MAIN:

            drawMainScreen();

            break;

        case SCREEN_MODE:

            drawModeScreen();

            break;

        case SCREEN_CURRENT:

            drawCurrentScreen();

            break;

        case SCREEN_VOLTAGE:

            drawVoltageScreen();

            break;

        case SCREEN_DURATION:

            drawDurationScreen();

            break;

        case SCREEN_CONFIRM:

            drawConfirmScreen();

            break;

        case SCREEN_INVALID:

            drawInvalidScreen();

            break;

        case SCREEN_RUNNING:

            drawRunningScreen();

            break;

        case SCREEN_COMPLETE:

            drawCompleteScreen();

            break;

        case SCREEN_FAULT:

            drawFaultScreen();

            break;
    }
}
