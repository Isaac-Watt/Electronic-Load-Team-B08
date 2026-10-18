#include <SPI.h>
#include <TFT_22_ILI9225.h>
#include <math.h>

#define TFT_RST   21
#define TFT_RS    13
#define TFT_CS    10
#define TFT_MOSI  11
#define TFT_SCK   12
#define TFT_LED   0

#define VOLTAGE_ADC_PIN     18
#define CURRENT_ADC_PIN     17
#define THERMISTOR_ADC_PIN  16

#define NUM_SAMPLES 32

const float VOLTAGE_ADC_CAL_1 = 1.461;
const float VOLTAGE_REAL_CAL_1 = 3.300;

const float VOLTAGE_ADC_CAL_2 = 3.010;
const float VOLTAGE_REAL_CAL_2 = 11.000;

const float ACS_ZERO_VOLTAGE = 1.700;
const float ACS_EFFECTIVE_SENSITIVITY = 0.033;
const float CURRENT_DEADBAND = 0.10;

const float THERMISTOR_NOMINAL_RESISTANCE = 10000.0;
const float THERMISTOR_NOMINAL_TEMPERATURE = 25.0;
const float THERMISTOR_BETA = 3950.0;
const float SERIES_RESISTOR = 10000.0;
const float SUPPLY_VOLTAGE = 3.3;

SPIClass spi = SPIClass(FSPI);

TFT_22_ILI9225 tft(
  TFT_RST,
  TFT_RS,
  TFT_CS,
  TFT_LED
);

float voltageADC = 0.0;
float actualVoltage = 0.0;

float currentADC = 0.0;
float actualCurrent = 0.0;

float temperatureC = 0.0;

float readADCVoltage(int pin)
{
  uint32_t totalMillivolts = 0;

  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    totalMillivolts += analogReadMilliVolts(pin);
    delay(2);
  }

  float averageMillivolts =
    (float)totalMillivolts / NUM_SAMPLES;

  return averageMillivolts / 1000.0;
}

float calculateActualVoltage(float adcVoltage)
{
  float slope =
    (VOLTAGE_REAL_CAL_2 - VOLTAGE_REAL_CAL_1) /
    (VOLTAGE_ADC_CAL_2 - VOLTAGE_ADC_CAL_1);

  float voltage =
    VOLTAGE_REAL_CAL_1 +
    slope *
    (adcVoltage - VOLTAGE_ADC_CAL_1);

  if (voltage < 0.0)
  {
    voltage = 0.0;
  }

  return voltage;
}

float calculateCurrent(float adcVoltage)
{
  float sensorDifference =
    adcVoltage - ACS_ZERO_VOLTAGE;

  float current =
    sensorDifference /
    ACS_EFFECTIVE_SENSITIVITY;

  current = fabs(current);

  if (current < CURRENT_DEADBAND)
  {
    current = 0.0;
  }

  return current;
}

float readThermistorResistance()
{
  float voltage =
    readADCVoltage(THERMISTOR_ADC_PIN);

  if (voltage <= 0.01)
  {
    return -1.0;
  }

  if (voltage >= SUPPLY_VOLTAGE - 0.01)
  {
    return -1.0;
  }

  float resistance =
    SERIES_RESISTOR *
    voltage /
    (SUPPLY_VOLTAGE - voltage);

  return resistance;
}

float readTemperature()
{
  float resistance =
    readThermistorResistance();

  if (resistance <= 0.0)
  {
    return NAN;
  }

  float T0 =
    THERMISTOR_NOMINAL_TEMPERATURE + 273.15;

  float temperatureKelvin =
    1.0 /
    (
      (1.0 / T0) +
      (1.0 / THERMISTOR_BETA) *
      log(
        resistance /
        THERMISTOR_NOMINAL_RESISTANCE
      )
    );

  float temperature =
    temperatureKelvin - 273.15;

  return temperature;
}

void drawMonitorScreen()
{
  tft.clear();

  tft.fillRectangle(
    0,
    0,
    175,
    25,
    COLOR_BLUE
  );

  tft.setFont(Terminal6x8);

  tft.drawText(
    15,
    8,
    "ELECTRONIC LOAD",
    COLOR_WHITE
  );

  tft.drawText(
    10,
    38,
    "VOLTAGE",
    COLOR_WHITE
  );

  tft.drawText(
    10,
    93,
    "CURRENT",
    COLOR_WHITE
  );

  tft.drawText(
    10,
    148,
    "TEMPERATURE",
    COLOR_WHITE
  );

  tft.drawText(
    8,
    205,
    "V18  I17  TEMP16",
    COLOR_YELLOW
  );
}

void updateVoltageDisplay(float voltage)
{
  char buffer[20];

  tft.fillRectangle(
    10,
    52,
    165,
    82,
    COLOR_BLACK
  );

  snprintf(
    buffer,
    sizeof(buffer),
    "%.2f V",
    voltage
  );

  tft.setFont(Terminal11x16);

  tft.drawText(
    40,
    57,
    String(buffer),
    COLOR_GREEN
  );

  tft.setFont(Terminal6x8);
}

void updateCurrentDisplay(float current)
{
  char buffer[20];

  tft.fillRectangle(
    10,
    107,
    165,
    137,
    COLOR_BLACK
  );

  snprintf(
    buffer,
    sizeof(buffer),
    "%.2f A",
    current
  );

  tft.setFont(Terminal11x16);

  tft.drawText(
    40,
    112,
    String(buffer),
    COLOR_GREEN
  );

  tft.setFont(Terminal6x8);
}

void updateTemperatureDisplay(float temperature)
{
  char buffer[20];

  tft.fillRectangle(
    10,
    162,
    165,
    195,
    COLOR_BLACK
  );

  tft.setFont(Terminal11x16);

  if (isnan(temperature))
  {
    tft.drawText(
      30,
      167,
      "FAULT",
      COLOR_RED
    );
  }
  else
  {
    snprintf(
      buffer,
      sizeof(buffer),
      "%.1f C",
      temperature
    );

    tft.drawText(
      40,
      167,
      String(buffer),
      COLOR_GREEN
    );
  }

  tft.setFont(Terminal6x8);
}

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();

  Serial.println(
    "================================"
  );

  Serial.println(
    "ELECTRONIC LOAD SENSOR TEST"
  );

  Serial.println(
    "================================"
  );

  Serial.println();

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

  spi.begin(
    TFT_SCK,
    -1,
    TFT_MOSI,
    TFT_CS
  );

  delay(200);

  tft.begin(spi);

  tft.setFont(Terminal6x8);

  tft.setOrientation(0);

  drawMonitorScreen();

  Serial.println("Display initialized.");

  Serial.println();

  Serial.println("ADC inputs:");

  Serial.println(
    "Voltage     -> GPIO18"
  );

  Serial.println(
    "Current     -> GPIO17"
  );

  Serial.println(
    "Thermistor  -> GPIO16"
  );

  Serial.println();

  Serial.println(
    "TFT reset   -> GPIO21"
  );

  Serial.println();

  Serial.println(
    "Voltage calibration:"
  );

  Serial.println(
    "1.461 V ADC = 3.30 V"
  );

  Serial.println(
    "3.010 V ADC = 11.00 V"
  );

  Serial.println();

  Serial.println(
    "Current zero = 1.700 V"
  );

  Serial.println(
    "Current effective sensitivity = 0.033 V/A"
  );

  Serial.println();
}

void loop()
{
  voltageADC =
    readADCVoltage(
      VOLTAGE_ADC_PIN
    );

  actualVoltage =
    calculateActualVoltage(
      voltageADC
    );

  currentADC =
    readADCVoltage(
      CURRENT_ADC_PIN
    );

  actualCurrent =
    calculateCurrent(
      currentADC
    );

  temperatureC =
    readTemperature();

  Serial.println(
    "--------------------------------"
  );

  Serial.print(
    "Voltage ADC: "
  );

  Serial.print(
    voltageADC,
    3
  );

  Serial.print(
    " V  ->  "
  );

  Serial.print(
    actualVoltage,
    2
  );

  Serial.println(
    " V"
  );

  Serial.print(
    "Current ADC: "
  );

  Serial.print(
    currentADC,
    3
  );

  Serial.print(
    " V  ->  "
  );

  Serial.print(
    actualCurrent,
    2
  );

  Serial.println(
    " A"
  );

  Serial.print(
    "Temperature: "
  );

  if (isnan(temperatureC))
  {
    Serial.println(
      "SENSOR FAULT"
    );
  }
  else
  {
    Serial.print(
      temperatureC,
      1
    );

    Serial.println(
      " C"
    );
  }

  updateVoltageDisplay(
    actualVoltage
  );

  updateCurrentDisplay(
    actualCurrent
  );

  updateTemperatureDisplay(
    temperatureC
  );

  delay(500);
}