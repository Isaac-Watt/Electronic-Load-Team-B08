#include <SPI.h>
#include <TFT_22_ILI9225.h>


#define TFT_RST   16
#define TFT_RS    13
#define TFT_CS    10
#define TFT_MOSI  11
#define TFT_SCK   12
#define TFT_LED   0

#define VOLTAGE_ADC_PIN 18


#define NUM_SAMPLES 32

SPIClass spi = SPIClass(FSPI);

TFT_22_ILI9225 tft(
  TFT_RST,
  TFT_RS,
  TFT_CS,
  TFT_LED
);

float measuredVoltage = 0.0;
float previousVoltage = -1.0;

float readVoltage()
{
  uint32_t adcTotal = 0;


  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    adcTotal += analogRead(VOLTAGE_ADC_PIN);

    delay(2);
  }

  // Average ADC reading
  float adcAverage =
    (float)adcTotal / NUM_SAMPLES;

  // Convert 12-bit ADC reading to 0-3.3 V
  float voltage =
    (adcAverage / 4095.0) * 3.3;

  return voltage;
}

void drawVoltageScreen()
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
    8,
    8,
    "VOLTAGE MONITOR",
    COLOR_WHITE
  );

  // Label
  tft.drawText(
    45,
    60,
    "ADC VOLTAGE",
    COLOR_WHITE
  );

  tft.drawText(
    30,
    145,
    "ADC INPUT: 0-3.3 V",
    COLOR_YELLOW
  );

  tft.drawText(
    25,
    165,
    "32 SAMPLE AVERAGE",
    COLOR_YELLOW
  );
}


void updateVoltageDisplay(float voltage)
{
  char buffer[20];


  tft.fillRectangle(
    25,
    85,
    155,
    120,
    COLOR_BLACK
  );

  snprintf(
    buffer,
    sizeof(buffer),
    "%.3f V",
    voltage
  );

  tft.setFont(Terminal11x16);

  tft.drawText(
    50,
    95,
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
  Serial.println("Starting Voltage Monitor...");

  analogReadResolution(12);

  analogSetPinAttenuation(
    VOLTAGE_ADC_PIN,
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

  
  drawVoltageScreen();

  Serial.println("Display initialized");
  Serial.println("Voltage monitoring started");
}

void loop()
{
  
  measuredVoltage = readVoltage();

  Serial.print("ADC Voltage: ");

  Serial.print(
    measuredVoltage,
    3
  );

  Serial.println(" V");


  updateVoltageDisplay(
    measuredVoltage
  );

  
  delay(500);
}
