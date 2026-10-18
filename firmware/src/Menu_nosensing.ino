#include <SPI.h>
#include <TFT_22_ILI9225.h>

#define TFT_RST   16
#define TFT_RS    13
#define TFT_CS    10
#define TFT_MOSI  11
#define TFT_SCK   12
#define TFT_LED   0

#define BTN_UP       17
#define BTN_DOWN     18
#define BTN_BACK     21
#define BTN_SELECT   1

SPIClass spi = SPIClass(FSPI);

TFT_22_ILI9225 tft(
  TFT_RST,
  TFT_RS,
  TFT_CS,
  TFT_LED
);

enum LoadMode
{
  MODE_CC,
  MODE_CV
};

LoadMode loadMode = MODE_CC;

float currentSetpoint = 1.00;
float voltageSetpoint = 5.0;

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
  0.1,
  1.0,
  5.0
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

enum ScreenState
{
  SCREEN_MAIN,
  SCREEN_MODE,
  SCREEN_CURRENT,
  SCREEN_VOLTAGE,
  SCREEN_DURATION,
  SCREEN_CONFIRM,
  SCREEN_RUNNING
};

ScreenState screen = SCREEN_MAIN;

int menuIndex = 0;

bool buttonPressed(int pin)
{
  if (digitalRead(pin) == LOW)
  {
    delay(25);

    if (digitalRead(pin) == LOW)
    {
      while (digitalRead(pin) == LOW)
      {
        delay(5);
      }

      delay(20);

      return true;
    }
  }

  return false;
}

int detectClick(int pin)
{
  if (digitalRead(pin) != LOW)
  {
    return 0;
  }

  delay(25);

  if (digitalRead(pin) != LOW)
  {
    return 0;
  }

  while (digitalRead(pin) == LOW)
  {
    delay(5);
  }

  unsigned long startTime = millis();

  while (millis() - startTime < 280)
  {
    if (digitalRead(pin) == LOW)
    {
      delay(25);

      if (digitalRead(pin) == LOW)
      {
        while (digitalRead(pin) == LOW)
        {
          delay(5);
        }

        delay(20);

        return 2;
      }
    }

    delay(5);
  }

  return 1;
}

void drawHeader(String title)
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
    title,
    COLOR_WHITE
  );
}

void drawMainMenu()
{
  drawHeader("DC ELECTRONIC LOAD");

  String items[4] =
  {
    "Mode",
    "Setpoint",
    "Duration",
    "Start Test"
  };

  for (int i = 0; i < 4; i++)
  {
    int y = 40 + (i * 28);

    if (i == menuIndex)
    {
      tft.fillRectangle(
        5,
        y - 4,
        170,
        y + 14,
        COLOR_BLUE
      );

      tft.drawText(
        12,
        y,
        "> " + items[i],
        COLOR_WHITE
      );
    }
    else
    {
      tft.drawText(
        12,
        y,
        "  " + items[i],
        COLOR_WHITE
      );
    }
  }

  char buffer[30];

  if (loadMode == MODE_CC)
  {
    snprintf(
      buffer,
      sizeof(buffer),
      "CC: %.2f A",
      currentSetpoint
    );
  }
  else
  {
    snprintf(
      buffer,
      sizeof(buffer),
      "CV: %.1f V",
      voltageSetpoint
    );
  }

  tft.drawText(
    8,
    165,
    String(buffer),
    COLOR_GREEN
  );

  snprintf(
    buffer,
    sizeof(buffer),
    "Time: %d min",
    durationMinutes
  );

  tft.drawText(
    8,
    182,
    String(buffer),
    COLOR_GREEN
  );

  tft.drawText(
    8,
    205,
    "UP/DN + LEFT/RIGHT",
    COLOR_YELLOW
  );
}

void drawModeScreen()
{
  drawHeader("SELECT MODE");

  if (loadMode == MODE_CC)
  {
    tft.fillRectangle(
      8,
      52,
      168,
      78,
      COLOR_BLUE
    );

    tft.drawText(
      15,
      61,
      "> Constant Current",
      COLOR_WHITE
    );
  }
  else
  {
    tft.drawText(
      15,
      61,
      "  Constant Current",
      COLOR_WHITE
    );
  }

  if (loadMode == MODE_CV)
  {
    tft.fillRectangle(
      8,
      92,
      168,
      118,
      COLOR_BLUE
    );

    tft.drawText(
      15,
      101,
      "> Constant Voltage",
      COLOR_WHITE
    );
  }
  else
  {
    tft.drawText(
      15,
      101,
      "  Constant Voltage",
      COLOR_WHITE
    );
  }

  tft.drawText(
    10,
    175,
    "UP/DOWN = CHANGE",
    COLOR_YELLOW
  );

  tft.drawText(
    10,
    195,
    "RIGHT=OK LEFT=BACK",
    COLOR_YELLOW
  );
}

void drawCurrentScreen()
{
  drawHeader("SET CURRENT");

  char buffer[30];

  snprintf(
    buffer,
    sizeof(buffer),
    "%.2f A",
    currentSetpoint
  );

  tft.setFont(Terminal11x16);

  tft.drawText(
    55,
    55,
    String(buffer),
    COLOR_GREEN
  );

  tft.setFont(Terminal6x8);

  snprintf(
    buffer,
    sizeof(buffer),
    "Step: %.2f A",
    currentSteps[currentStepIndex]
  );

  tft.drawText(
    45,
    95,
    String(buffer),
    COLOR_YELLOW
  );

  tft.drawText(
    15,
    125,
    "UP/DOWN = ADJUST",
    COLOR_WHITE
  );

  tft.drawText(
    15,
    145,
    "DOUBLE UP = STEP",
    COLOR_WHITE
  );

  tft.drawText(
    10,
    195,
    "RIGHT=OK LEFT=BACK",
    COLOR_YELLOW
  );
}

void drawVoltageScreen()
{
  drawHeader("SET VOLTAGE");

  char buffer[30];

  snprintf(
    buffer,
    sizeof(buffer),
    "%.1f V",
    voltageSetpoint
  );

  tft.setFont(Terminal11x16);

  tft.drawText(
    55,
    55,
    String(buffer),
    COLOR_GREEN
  );

  tft.setFont(Terminal6x8);

  snprintf(
    buffer,
    sizeof(buffer),
    "Step: %.1f V",
    voltageSteps[voltageStepIndex]
  );

  tft.drawText(
    45,
    95,
    String(buffer),
    COLOR_YELLOW
  );

  tft.drawText(
    15,
    125,
    "UP/DOWN = ADJUST",
    COLOR_WHITE
  );

  tft.drawText(
    15,
    145,
    "DOUBLE UP = STEP",
    COLOR_WHITE
  );

  tft.drawText(
    10,
    195,
    "RIGHT=OK LEFT=BACK",
    COLOR_YELLOW
  );
}

void drawDurationScreen()
{
  drawHeader("TEST DURATION");

  char buffer[30];

  snprintf(
    buffer,
    sizeof(buffer),
    "%d min",
    durationMinutes
  );

  tft.setFont(Terminal11x16);

  tft.drawText(
    50,
    55,
    String(buffer),
    COLOR_GREEN
  );

  tft.setFont(Terminal6x8);

  snprintf(
    buffer,
    sizeof(buffer),
    "Step: %d min",
    timeSteps[timeStepIndex]
  );

  tft.drawText(
    45,
    95,
    String(buffer),
    COLOR_YELLOW
  );

  tft.drawText(
    15,
    125,
    "UP/DOWN = ADJUST",
    COLOR_WHITE
  );

  tft.drawText(
    15,
    145,
    "DOUBLE UP = STEP",
    COLOR_WHITE
  );

  tft.drawText(
    10,
    195,
    "RIGHT=OK LEFT=BACK",
    COLOR_YELLOW
  );
}

void drawConfirmScreen()
{
  drawHeader("CONFIRM TEST");

  char buffer[30];

  if (loadMode == MODE_CC)
  {
    tft.drawText(
      15,
      50,
      "Mode: CC",
      COLOR_WHITE
    );

    snprintf(
      buffer,
      sizeof(buffer),
      "Current: %.2f A",
      currentSetpoint
    );
  }
  else
  {
    tft.drawText(
      15,
      50,
      "Mode: CV",
      COLOR_WHITE
    );

    snprintf(
      buffer,
      sizeof(buffer),
      "Voltage: %.1f V",
      voltageSetpoint
    );
  }

  tft.drawText(
    15,
    75,
    String(buffer),
    COLOR_WHITE
  );

  snprintf(
    buffer,
    sizeof(buffer),
    "Duration: %d min",
    durationMinutes
  );

  tft.drawText(
    15,
    100,
    String(buffer),
    COLOR_WHITE
  );

  tft.fillRectangle(
    10,
    140,
    165,
    165,
    COLOR_BLUE
  );

  tft.drawText(
    25,
    149,
    "RIGHT = START",
    COLOR_WHITE
  );

  tft.drawText(
    25,
    185,
    "LEFT = CANCEL",
    COLOR_YELLOW
  );
}

void drawRunningScreen()
{
  drawHeader("TEST RUNNING");

  tft.setFont(Terminal11x16);

  tft.drawText(
    55,
    42,
    "ACTIVE",
    COLOR_GREEN
  );

  tft.setFont(Terminal6x8);

  char buffer[30];

  if (loadMode == MODE_CC)
  {
    snprintf(
      buffer,
      sizeof(buffer),
      "Target: %.2f A",
      currentSetpoint
    );
  }
  else
  {
    snprintf(
      buffer,
      sizeof(buffer),
      "Target: %.1f V",
      voltageSetpoint
    );
  }

  tft.drawText(
    20,
    85,
    String(buffer),
    COLOR_WHITE
  );

  snprintf(
    buffer,
    sizeof(buffer),
    "Duration: %d min",
    durationMinutes
  );

  tft.drawText(
    20,
    110,
    String(buffer),
    COLOR_WHITE
  );

  tft.drawText(
    20,
    185,
    "LEFT = STOP TEST",
    COLOR_RED
  );
}

void redraw()
{
  tft.setFont(Terminal6x8);

  switch (screen)
  {
    case SCREEN_MAIN:
      drawMainMenu();
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

    case SCREEN_RUNNING:
      drawRunningScreen();
      break;
  }
}

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println("Starting UI...");

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

  spi.begin(
    TFT_SCK,
    -1,
    TFT_MOSI,
    TFT_CS
  );

  Serial.println("SPI initialized");

  delay(200);

  tft.begin(spi);

  Serial.println("Display initialized");

  tft.setFont(Terminal6x8);

  tft.setOrientation(0);

  drawMainMenu();

  Serial.println("Menu ready");
}

void loop()
{
  if (screen == SCREEN_MAIN)
  {
    if (buttonPressed(BTN_UP))
    {
      menuIndex--;

      if (menuIndex < 0)
      {
        menuIndex = 3;
      }

      redraw();
    }

    else if (buttonPressed(BTN_DOWN))
    {
      menuIndex++;

      if (menuIndex > 3)
      {
        menuIndex = 0;
      }

      redraw();
    }

    else if (buttonPressed(BTN_SELECT))
    {
      if (menuIndex == 0)
      {
        screen = SCREEN_MODE;
      }

      else if (menuIndex == 1)
      {
        if (loadMode == MODE_CC)
        {
          screen = SCREEN_CURRENT;
        }
        else
        {
          screen = SCREEN_VOLTAGE;
        }
      }

      else if (menuIndex == 2)
      {
        screen = SCREEN_DURATION;
      }

      else if (menuIndex == 3)
      {
        screen = SCREEN_CONFIRM;
      }

      redraw();
    }
  }

  else if (screen == SCREEN_MODE)
  {
    if (
      buttonPressed(BTN_UP) ||
      buttonPressed(BTN_DOWN)
    )
    {
      if (loadMode == MODE_CC)
      {
        loadMode = MODE_CV;
      }
      else
      {
        loadMode = MODE_CC;
      }

      redraw();
    }

    else if (buttonPressed(BTN_SELECT))
    {
      screen = SCREEN_MAIN;

      redraw();
    }

    else if (buttonPressed(BTN_BACK))
    {
      screen = SCREEN_MAIN;

      redraw();
    }
  }

  else if (screen == SCREEN_CURRENT)
  {
    int upClick = detectClick(BTN_UP);

    if (upClick == 2)
    {
      currentStepIndex++;

      if (currentStepIndex > 2)
      {
        currentStepIndex = 0;
      }

      Serial.print("Current step changed to: ");
      Serial.println(currentSteps[currentStepIndex]);

      redraw();
    }

    else if (upClick == 1)
    {
      currentSetpoint += currentSteps[currentStepIndex];

      if (currentSetpoint > 30.0)
      {
        currentSetpoint = 30.0;
      }

      redraw();
    }

    else if (buttonPressed(BTN_DOWN))
    {
      currentSetpoint -= currentSteps[currentStepIndex];

      if (currentSetpoint < 0.0)
      {
        currentSetpoint = 0.0;
      }

      redraw();
    }

    else if (buttonPressed(BTN_SELECT))
    {
      screen = SCREEN_MAIN;

      redraw();
    }

    else if (buttonPressed(BTN_BACK))
    {
      screen = SCREEN_MAIN;

      redraw();
    }
  }

  else if (screen == SCREEN_VOLTAGE)
  {
    int upClick = detectClick(BTN_UP);

    if (upClick == 2)
    {
      voltageStepIndex++;

      if (voltageStepIndex > 2)
      {
        voltageStepIndex = 0;
      }

      Serial.print("Voltage step changed to: ");
      Serial.println(voltageSteps[voltageStepIndex]);

      redraw();
    }

    else if (upClick == 1)
    {
      voltageSetpoint += voltageSteps[voltageStepIndex];

      if (voltageSetpoint > 30.0)
      {
        voltageSetpoint = 30.0;
      }

      redraw();
    }

    else if (buttonPressed(BTN_DOWN))
    {
      voltageSetpoint -= voltageSteps[voltageStepIndex];

      if (voltageSetpoint < 0.0)
      {
        voltageSetpoint = 0.0;
      }

      redraw();
    }

    else if (buttonPressed(BTN_SELECT))
    {
      screen = SCREEN_MAIN;

      redraw();
    }

    else if (buttonPressed(BTN_BACK))
    {
      screen = SCREEN_MAIN;

      redraw();
    }
  }

  else if (screen == SCREEN_DURATION)
  {
    int upClick = detectClick(BTN_UP);

    if (upClick == 2)
    {
      timeStepIndex++;

      if (timeStepIndex > 3)
      {
        timeStepIndex = 0;
      }

      Serial.print("Time step changed to: ");
      Serial.println(timeSteps[timeStepIndex]);

      redraw();
    }

    else if (upClick == 1)
    {
      durationMinutes += timeSteps[timeStepIndex];

      if (durationMinutes > 999)
      {
        durationMinutes = 999;
      }

      redraw();
    }

    else if (buttonPressed(BTN_DOWN))
    {
      durationMinutes -= timeSteps[timeStepIndex];

      if (durationMinutes < 1)
      {
        durationMinutes = 1;
      }

      redraw();
    }

    else if (buttonPressed(BTN_SELECT))
    {
      screen = SCREEN_MAIN;

      redraw();
    }

    else if (buttonPressed(BTN_BACK))
    {
      screen = SCREEN_MAIN;

      redraw();
    }
  }

  else if (screen == SCREEN_CONFIRM)
  {
    if (buttonPressed(BTN_SELECT))
    {
      Serial.println("TEST START");

      screen = SCREEN_RUNNING;

      redraw();
    }

    else if (buttonPressed(BTN_BACK))
    {
      screen = SCREEN_MAIN;

      redraw();
    }
  }

  else if (screen == SCREEN_RUNNING)
  {
    if (buttonPressed(BTN_BACK))
    {
      Serial.println("TEST STOP");

      screen = SCREEN_MAIN;

      redraw();
    }
  }
}
