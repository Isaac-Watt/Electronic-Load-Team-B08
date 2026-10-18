#include <SPI.h>
#include <TFT_22_ILI9225.h>
#include <math.h>

#define TFT_RST   16
#define TFT_RS    13
#define TFT_CS    10
#define TFT_MOSI  11
#define TFT_SCK   12
#define TFT_LED   0

#define VOLTAGE_ADC_PIN 18
#define CURRENT_ADC_PIN 17

#define NUM_SAMPLES 32

#define VOLTAGE_SLOPE   4.971
#define VOLTAGE_OFFSET -3.963

#define ACS_ZERO_VOLTAGE  1.700
#define ACS_SENSITIVITY   0.066

#define CURRENT_DEADBAND  0.10

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

float readADCVoltage(int pin)
{
  uint32_t adcTotal = 0;

  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    adcTotal += analogRead(pin);
    delay(2);
  }

  float adcAverage =
    (float)adcTotal / NUM_SAMPLES;

  float adcVoltage =
    (adcAverage / 4095.0) * 3.3;

  return adcVoltage;
}

float calculateActualVoltage(float adcV)
{
  float voltage =
    (VOLTAGE_SLOPE * adcV) + VOLTAGE_OFFSET;

  if (voltage < 0.0)
  {
    voltage = 0.0;
  }

  return voltage;
}

float calculateCurrent(float adcV)
{
  float current =
    (adcV - ACS_ZERO_VOLTAGE)
    / ACS_SENSITIVITY;

  current = fabs(current);

  if (current < CURRENT_DEADBAND)
  {
    current = 0.0;
  }

  return current;
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
    45,
    "VOLTAGE",
    COLOR_WHITE
  );

  tft.drawText(
    10,
    110,
    "CURRENT",
    COLOR_WHITE
  );

  tft.drawText(
    10,
    185,
    "V: GPIO18  I: GPIO17",
    COLOR_YELLOW
  );

  tft.drawText(
    10,
    200,
    "32 SAMPLE AVERAGE",
    COLOR_YELLOW
  );
}

void updateVoltageDisplay(float voltage)
{
  char buffer[20];

  tft.fillRectangle(
    10,
    60,
    165,
    95,
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
    70,
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
    125,
    165,
    160,
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
    135,
    String(buffer),
    COLOR_GREEN
  );

  tft.setFont(Terminal6x8);
}

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("Electronic Load Monitor");
  Serial.println("==============================");
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

  Serial.println("Display initialized");
  Serial.println("Voltage sensing: GPIO18");
  Serial.println("Current sensing: GPIO17");
  Serial.println("ACS712 zero point: 1.700 V");
  Serial.println("ACS712 sensitivity: 0.066 V/A");
  Serial.println();
}

void loop()
{
  voltageADC =
    readADCVoltage(VOLTAGE_ADC_PIN);

  actualVoltage =
    calculateActualVoltage(voltageADC);

  currentADC =
    readADCVoltage(CURRENT_ADC_PIN);

  actualCurrent =
    calculateCurrent(currentADC);

  Serial.print("Voltage ADC: ");
  Serial.print(voltageADC, 3);
  Serial.print(" V");

  Serial.print("  |  Voltage: ");
  Serial.print(actualVoltage, 2);
  Serial.print(" V");

  Serial.print("  |  ACS712 ADC: ");
  Serial.print(currentADC, 3);
  Serial.print(" V");

  Serial.print("  |  Current: ");
  Serial.print(actualCurrent, 2);
  Serial.println(" A");

  updateVoltageDisplay(actualVoltage);
  updateCurrentDisplay(actualCurrent);

  delay(500);
}
