#include "MAX31855soft.h" //https://github.com/enjoyneering/MAX31855
#include <U8g2lib.h> //https://github.com/olikraus/u8g2
#include "splash.h"
#include "helper.h"
#include "FlashStorage.h"
#include "PID_v1.h"
#include "pidautotuner.h"
#include "variables.h"
#include "menu.h"
#include "serial.h"

double shouldBeTemp;

uint8_t fanValue;
uint8_t heaterValueChan0;

double SetpointChan0;
double InputChan0;
double OutputChan0;

uint8_t heaterValueChan1;

double SetpointChan1;
double InputChan1;
double OutputChan1;

PID3_t heaterPIDChan0 = { PID_KP_PREHEAT_C0, PID_KI_PREHEAT_C0, PID_KD_PREHEAT_C0, PID_KP_SOAK_C0, PID_KI_SOAK_C0, PID_KD_SOAK_C0, PID_KP_REFLOW_C0, PID_KI_REFLOW_C0, PID_KD_REFLOW_C0 };
PID3_t heaterPIDChan1 = { PID_KP_PREHEAT_C1, PID_KI_PREHEAT_C1, PID_KD_PREHEAT_C1, PID_KP_SOAK_C1, PID_KI_SOAK_C1, PID_KD_SOAK_C1, PID_KP_REFLOW_C1, PID_KI_REFLOW_C1, PID_KD_REFLOW_C1 };
PID_t fanPID    = { 10.00, 0.03, 10.00 }; //{ 1.00, 0.03, 10.00 };

PID PIDChan0(&InputChan0, &OutputChan0, &SetpointChan0, heaterPIDChan0.Kp_PREHEAT, heaterPIDChan0.Ki_PREHEAT, heaterPIDChan0.Kd_PREHEAT, DIRECT);

PIDAutotuner tunerChan0 = PIDAutotuner();

PID PIDChan1(&InputChan1, &OutputChan1, &SetpointChan1, heaterPIDChan1.Kp_PREHEAT, heaterPIDChan1.Ki_PREHEAT, heaterPIDChan1.Kd_PREHEAT, DIRECT);

PIDAutotuner tunerChan1 = PIDAutotuner();

StoredVar storedVar;

int currentState = IDLEM;

double rampRateChan0 = 0;
double rampRateChan1 = 0;

volatile int frontState = 0;
volatile int backState = 0;
volatile int convectionState = 0;
volatile int exhaustState = 0;

extern U8G2_KS0108_128X64_F u8g2;
extern U8G2LOG u8g2log;
extern uint8_t u8log_buffer[];
extern int errorTimeout;

bool beepState = true;

bool saved = true;
int writeTimeout = -1;
FlashStorage(storedVarFlash, StoredVar);

MAX31855soft MAX31855_0(CSU2, SO, SCK);
MAX31855soft MAX31855_1(CSU3, SO, SCK);

double tc0History[NUMBER_OF_TEMP_AVERAGES] = {0};
double tc1History[NUMBER_OF_TEMP_AVERAGES] = {0};

int averagesCount = 0;

int tempHistoryHead = 0;

double tc0Temp; //Back by default
double tc1Temp; //Front by default
double backTemp;
double frontTemp;
int tcState = 0; //0 normal (Back == TC0, Front == TC1), 1 swapped (Back == TC1, Front == TC0), 2 averaging (Back=Front=(TC0+TC1)/2)

double tc0PrevTemp;
double tc1PrevTemp;

int tc0Detect = MAX31855_THERMOCOUPLE_OK;
int tc1Detect = MAX31855_THERMOCOUPLE_OK;

int currentProfile = 1;
ReflowProfile profiles[MAX_PROFILES + 1]; //Don't use 0 profile

double boardTemp;

int currentMenuID = 0;

unsigned long reflowStartTime = 0;

#define buttonStateTimeout 5

volatile int buttonStatePLUS = 0;
volatile int buttonStateMINUS = 0;
volatile int buttonStateBACK = 0;
volatile int buttonStateOK = 0;

int menuState = 0;
int prevMenuState = 0;
int buttonBACK = 1;
int prevButtonBACK = 1;
unsigned long previousTimeUpdateTemp = 0;
unsigned long previousTimeReadTemp = 0;

int enableMenu = 1;

bool celsiusMode = true; //true is celcius, false is farenheit

int reflow = 0;
unsigned long previousTimeReflowDisplayUpdate = 0;
unsigned long currentReflowSeconds = 0;
unsigned long previousTempDisplayUpdate = 0;

unsigned long prevTempLoopMillis;
unsigned long currentTempLoopMillis;

unsigned long secondsMillisStartChan0;
unsigned long secondsMillisStartChan1;

unsigned long startStateMillis;

int previousState = IDLEM;

uint32_t lastRampTicks;

double startTemp = 0;

bool valueChanged = false;
int profileValueBeingEdited = 0;

int previousSavedProfile;

int currentPIDEditSelection = 0;

unsigned long pollRate = 1000;

double previousRateDisplayTemp0 = 0;

double previousRateDisplayTemp1 = 0;

boolean tuning = false;

boolean currentStateAutotuned = false;

boolean debug = false;

void plusPress()
{
  if (buttonStatePLUS == 0)
    buttonStatePLUS = buttonStateTimeout;
}

void minusPress()
{
  if (buttonStateMINUS == 0)
    buttonStateMINUS = buttonStateTimeout;
}

void okPress()
{
  if (buttonStateOK == 0)
    buttonStateOK = buttonStateTimeout;
}

void zeroCrossing()
{
  int state = digitalRead(ZER_D);

  if (state == 0) //We're in falling
  {
    if (frontState == 1) //Turn on
    {
      digitalWrite(FRONT_HEATER, 0);
    }
    if (backState == 1) //Turn on
    {
      digitalWrite(BACK_HEATER, 0);
    }
    if (convectionState == 1) //Turn on
    {
      digitalWrite(CONVECTION, 0);
    }
    if (exhaustState == 1) //Turn on
    {
      digitalWrite(EXHAUST, 0);
    }
  } else { //We're in rising
    if (frontState == 0)
    { //Turn off
      digitalWrite(FRONT_HEATER, 1);
    }
    if (backState == 0)
    { //Turn off
      digitalWrite(BACK_HEATER, 1);
    }
    if (convectionState == 0)
    { //Turn off
      digitalWrite(CONVECTION, 1);
    }
    if (exhaustState == 0)
    { //Turn off
      digitalWrite(EXHAUST, 1);
    }
  }
}

void loadSettings()
{
  for (int i = 0; i < MAX_PROFILES; i++)
  {
    profiles[i + 1].PreHtTemp = storedVar.profiles[i].PreHtTemp;
    profiles[i + 1].HeatTemp = storedVar.profiles[i].HeatTemp;
    profiles[i + 1].RefTemp = storedVar.profiles[i].RefTemp;
    profiles[i + 1].RefKpTemp = storedVar.profiles[i].RefKpTemp;
    profiles[i + 1].CoolTemp = storedVar.profiles[i].CoolTemp;

    profiles[i + 1].PreHtTime = storedVar.profiles[i].PreHtTime;
    profiles[i + 1].HeatTime = storedVar.profiles[i].HeatTime;
    profiles[i + 1].RefTime = storedVar.profiles[i].RefTime;
    profiles[i + 1].RefKpTime = storedVar.profiles[i].RefKpTime;
    profiles[i + 1].CoolTime = storedVar.profiles[i].CoolTime;
  }

  heaterPIDChan0.Kp_PREHEAT = storedVar.KpChan0_PREHEAT;
  heaterPIDChan0.Ki_PREHEAT = storedVar.KiChan0_PREHEAT;
  heaterPIDChan0.Kd_PREHEAT = storedVar.KdChan0_PREHEAT;
  heaterPIDChan1.Kp_PREHEAT = storedVar.KpChan1_PREHEAT;
  heaterPIDChan1.Ki_PREHEAT = storedVar.KiChan1_PREHEAT;
  heaterPIDChan1.Kd_PREHEAT = storedVar.KdChan1_PREHEAT;

  heaterPIDChan0.Kp_SOAK = storedVar.KpChan0_SOAK;
  heaterPIDChan0.Ki_SOAK = storedVar.KiChan0_SOAK;
  heaterPIDChan0.Kd_SOAK = storedVar.KdChan0_SOAK;
  heaterPIDChan1.Kp_SOAK = storedVar.KpChan1_SOAK;
  heaterPIDChan1.Ki_SOAK = storedVar.KiChan1_SOAK;
  heaterPIDChan1.Kd_SOAK = storedVar.KdChan1_SOAK;

  heaterPIDChan0.Kp_REFLOW = storedVar.KpChan0_REFLOW;
  heaterPIDChan0.Ki_REFLOW = storedVar.KiChan0_REFLOW;
  heaterPIDChan0.Kd_REFLOW = storedVar.KdChan0_REFLOW;
  heaterPIDChan1.Kp_REFLOW = storedVar.KpChan1_REFLOW;
  heaterPIDChan1.Ki_REFLOW = storedVar.KiChan1_REFLOW;
  heaterPIDChan1.Kd_REFLOW = storedVar.KdChan1_REFLOW;

  tcState = storedVar.tcState;

  celsiusMode = storedVar.celsiusMode;

  beepState = storedVar.beepState;

  PIDChan0.SetTunings(heaterPIDChan0.Kp_PREHEAT, heaterPIDChan0.Ki_PREHEAT, heaterPIDChan0.Kd_PREHEAT);
  PIDChan1.SetTunings(heaterPIDChan1.Kp_PREHEAT, heaterPIDChan1.Ki_PREHEAT, heaterPIDChan1.Kd_PREHEAT);

  currentProfile = storedVar.profile;
}

void checkSerial()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '#')
    {
      /* Set bootloader check value and reboot */
      BOOT_MAGIC_VALUE = BOOT_MAGIC_BOOTLOADER_ENABLE;
      NVIC_SystemReset();//software reset
    }
    else if (c == 'R') //Command to reset for debug purposes
    {
      BOOT_MAGIC_VALUE = 0;
      NVIC_SystemReset();//software reset
    }
    else if (c == 'T') //Print Temps
    {
      if (celsiusMode) Serial.print(backTemp);
      else Serial.print(cToF(backTemp));
      Serial.print(",");
      if (celsiusMode) Serial.print(frontTemp);
      else Serial.println(cToF(frontTemp));
    }
    else if (c == ' ' || c == 'c' || c == 'C') //OK: Spacebar Pressed (or c)
    {
      buttonStateOK = 1; //Will trigger next loop
    }
    else if (c == 'b' || c == 'B' || c == 'v' || c == 'V') //Back: b pressed (or v)
    {
      buttonStateBACK = buttonStateTimeout; //Will trigger next loop (note this is handled differently to other buttons
    }
    else if (c == 'p' || c == 'P' || c == 'z' || c == 'Z') //Plus: p pressed (or z)
    {
      buttonStatePLUS = 1; //Will trigger next loop
    }
    else if (c == 'm' || c == 'M' || c == 'x' || c == 'X') //Minus: m pressed (or x)
    {
      buttonStateMINUS = 1; //Will trigger next loop
    }
    else if (c == '*')
    {
      debug = true;
    }
    else if (c == '8')
    {
      debug = false;
    }
    else if (c == 'A' && debug) //All outputs on for testing
    {
      frontState = 1;
      digitalWrite(FRONT_HEATER, 0);
      backState = 1;
      digitalWrite(BACK_HEATER, 0);
      convectionState = 1;
      digitalWrite(CONVECTION, 0);
      exhaustState = 1;
      digitalWrite(EXHAUST, 0);
      digitalWrite(SPK, 1);
    }
    else if (c == 'a' && debug) //All outputs off for testing
    {
      frontState = 0;
      digitalWrite(FRONT_HEATER, 1);
      backState = 0;
      digitalWrite(BACK_HEATER, 1);
      convectionState = 0;
      digitalWrite(CONVECTION, 1);
      exhaustState = 0;
      digitalWrite(EXHAUST, 1);
      digitalWrite(SPK, 0);
    }
   
  }
}

void setup()
{
  /* \/ \/ \/ Keep below here \/ \/ \/ */
  /* Protect against bad code requiring ICSP flashing, do not remove */
  /* This allows us to hold # on startup if we had a bad flash */
  Serial.begin(115200);
  int countdown = 1000;
  while (countdown > 0)
  {
    checkSerial();
    countdown--;
    delay(1);
  }
  /* /\ /\ /\ Keep above here /\ /\ /\ */

  setOutputs();

  u8g2.begin();
  //u8g2.clearBuffer();
  u8g2.drawXBM(0, 0, splash_width, splash_height, splash_bits);
  u8g2.sendBuffer();

  u8g2.setFont(u8g2_font_tom_thumb_4x6_mf); // set the font for the terminal window

  u8g2log.begin(U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer); // connect to u8g2, assign buffer
  u8g2log.setLineHeightOffset(0);                         // set extra space between lines in pixel, this can be negative
  //u8g2log.setRedrawMode(0);   // 0: Update screen with newline, 1: Update screen for every char

  attachInterrupt(digitalPinToInterrupt(PLUS), plusPress, FALLING);
  attachInterrupt(digitalPinToInterrupt(MINUS), minusPress, FALLING);
  attachInterrupt(digitalPinToInterrupt(OK), okPress, FALLING);

  //BACK button is not on interrupt

  attachInterrupt(digitalPinToInterrupt(ZER_D), zeroCrossing, CHANGE);

  /* start MAX31855 */
  MAX31855_0.begin();
  MAX31855_1.begin();

  /* Get EEPROM Settings */
  storedVar = storedVarFlash.read();
  if (storedVar.valid == false) //Should be false after new code write
  {
    restoreDefaults(&storedVar);
  }

  loadSettings();

  previousSavedProfile = currentProfile; //Used to reduce writes to flash on a common menu function

  PIDChan0.SetOutputLimits(0, 100); // max output 100%
  PIDChan0.SetMode(AUTOMATIC);

  PIDChan1.SetOutputLimits(0, 100); // max output 100%
  PIDChan1.SetMode(AUTOMATIC);

  delay(2000);

  u8g2.clearBuffer();

  drawScreen();
  drawMenu(0); //Draw main menu
  drawMenuBox(0);
  drawProfile(profiles, tc1Temp, currentProfile);

  u8g2.sendBuffer();
}

void drawMenu(int index)
{
  switch (index)
  {
    default: //Goto main menu
    case 0: //Main Menu
      printMainMenu(currentProfile);
      break;
    case 1: //Config Menu
      printConfigMenu(celsiusMode, tcState, beepState);
      break;
    case 2: //Profile Menu
      printProfileMenu();
      break;
    case 3: //Tools Menu
      printToolsMenu();
      break;
    case 4: //Start Has been Pressed

      printCurrentState(currentState);

      if (celsiusMode)
      {
        setCursor(intLen((int)shouldBeTemp), 1);
        u8g2.print((int)shouldBeTemp);
      }
      else
      {
        setCursor(intLen((int)cToF(shouldBeTemp)), 1);
        u8g2.print((int)cToF(shouldBeTemp));
      }

      if (celsiusMode) {
        if (((rampRateChan0 + rampRateChan1) / 2) < 0) //Negative
          setCursor(intLen((int)((rampRateChan0 + rampRateChan1) / 2)) + 4, 2);
        else
          setCursor(intLen((int)((rampRateChan0 + rampRateChan1) / 2)) + 3, 2);
        u8g2.print((rampRateChan0 + rampRateChan1) / 2);
      }
      else
      {
        if (cToF((rampRateChan0 + rampRateChan1) / 2) < 0)
          setCursor(intLen((int)cToF((rampRateChan0 + rampRateChan1) / 2)) + 4, 2);
        else
          setCursor(intLen((int)cToF((rampRateChan0 + rampRateChan1) / 2)) + 3, 2);
        u8g2.print(cToF((rampRateChan0 + rampRateChan1) / 2));
      }
      setCursor(intLen((int)currentReflowSeconds), 3);
      u8g2.print(currentReflowSeconds); //Degrees per second
      setCursor(4, 4);
      u8g2.print("Stop");
      break;

    case 5: //Edit Values
      printEditValues(profileValueBeingEdited, profiles, currentProfile, celsiusMode);
      break;

    case 6: //Test
      printTestMenu();
      break;

    case 7: //PID
      printCurrentPIDMenu(heaterPIDChan0, heaterPIDChan1, currentPIDEditSelection);
      break;
  }
}

void readTemps()
{
  tc0PrevTemp = tc0Temp;
  tc1PrevTemp = tc1Temp;

  if (averagesCount < NUMBER_OF_TEMP_AVERAGES) averagesCount++;
  int32_t rawData;

  rawData = MAX31855_0.readRawData();
  tc0Detect = MAX31855_0.detectThermocouple(rawData);

  if (tc0Detect == MAX31855_THERMOCOUPLE_OK)
  {
    double tc0Value;
    tc0Value = (double)MAX31855_0.getTemperature(rawData);
    tc0History[tempHistoryHead] = tc0Value;

    tc0Temp = avg(tc0History, averagesCount);
  } else {
    tc0Temp = 0;
  }

  rawData = MAX31855_1.readRawData();
  tc1Detect = MAX31855_1.detectThermocouple(rawData);

  if (tc1Detect == MAX31855_THERMOCOUPLE_OK)
  {
    double tc1Value;

    tc1Value = (double)MAX31855_1.getTemperature(rawData);

    tc1History[tempHistoryHead] = tc1Value;

    tc1Temp = avg(tc1History, averagesCount);

    boardTemp = MAX31855_1.getColdJunctionTemperature(rawData);
  } else {
    tc1Temp = 0;
  }

  tempHistoryHead++;
  if (tempHistoryHead >= NUMBER_OF_TEMP_AVERAGES)
    tempHistoryHead = 0;

  if (tcState == 0)
  {
    frontTemp = tc1Temp;
    backTemp = tc0Temp;
  }
  else if (tcState == 0)
  {
    frontTemp = tc0Temp;
    backTemp = tc1Temp;
  } else
  {
    frontTemp = (tc0Temp + tc1Temp) / 2;
    backTemp = frontTemp;
  }
}

void updateTemps()
{
  if (errorTimeout <= 0) {
    if (tc0Detect != MAX31855_THERMOCOUPLE_OK)
    {
      if (tc0Detect == MAX31855_THERMOCOUPLE_SHORT_TO_GND)
      {
        u8g2log.print("TC0:GND   ");
      } else {
        u8g2log.print("TC0:NC    ");
      }
    }
    else
    {
      u8g2log.print("TC0:");

      if (tc0Temp < 100)
        u8g2log.print(" ");

      if (celsiusMode) u8g2log.print(((double)((int)(tc0Temp * 100))) / 100);
      else u8g2log.print(((double)((int)(cToF(tc0Temp) * 100))) / 100);

      u8g2log.print(" ");
    }

    if (tc1Detect != MAX31855_THERMOCOUPLE_OK)
    {
      if (tc1Detect == MAX31855_THERMOCOUPLE_SHORT_TO_GND)
      {
        u8g2log.print("TC1:GND   ");
      } else {
        u8g2log.print("TC1:NC    ");
      }
    }
    else
    {
      u8g2log.print("TC1:");
      if (tc1Temp < 100)
        u8g2log.print(" ");

      if (celsiusMode) u8g2log.print(((double)((int)(tc1Temp * 100))) / 100);
      else u8g2log.print(((double)((int)(cToF(tc1Temp) * 100))) / 100);
      u8g2log.print(" ");

      u8g2log.print("TOP:");
      if (boardTemp < 100)
        u8g2log.print(" ");
      if (celsiusMode) u8g2log.print(((double)((int)(boardTemp * 100))) / 100);
      else u8g2log.print(((double)((int)(cToF(boardTemp) * 100))) / 100);
    }

    // Refresh the screen
    u8g2log.print("\r");
    u8g2.drawLog(0, 5, u8g2log);

    u8g2.updateDisplayArea(0, 0, 128, 6);
  } else {
    errorTimeout--;
  }
}

void disableReflow()
{
  reflow = 0;
  currentState = IDLEM;
  pollRate = 1000;
}

void startReflow()
{
  reflowStartTime = millis();
  previousTempDisplayUpdate = 0;
  reflow = 1;
  currentState = PREHT;

  shouldBeTemp = (int)((backTemp + frontTemp) / 2);
  rampRateChan0 = 0;
  rampRateChan1 = 0;

  previousRateDisplayTemp0 = backTemp;
  previousRateDisplayTemp1 = frontTemp;

  pollRate = 200;
  PIDChan0.SetSampleTime(200);
  PIDChan1.SetSampleTime(200);
}

void profileEdited()
{
  buttonStateOK = 0;
  buttonStatePLUS = 0;
  buttonStateMINUS = 0;
  valueChanged = true;
  refreshMenu();
  drawMenu(currentMenuID);
  drawMenuBox(menuState);
  u8g2.updateDisplay();
}

void loop()
{
  /* Back button not on interrupt */
  buttonBACK = digitalRead(BACK);
  if (buttonBACK == 0 && prevButtonBACK == 1 && buttonStateBACK == 0)
  {
    buttonStateBACK = buttonStateTimeout;
  }

  prevButtonBACK = buttonBACK;

  if (buttonStateMINUS == 1 && buttonStatePLUS == 1)
  {
    //Ignore
  }
  else if (buttonStateMINUS == 1)
  {
    if (debug) Serial.println("Minus Pressed");
    if (enableMenu == 1)
    {
      prevMenuState = menuState;

      if (currentMenuID == 5)
      {
        if (menuState == 2 && digitalRead(OK) == 0)
        {
          delay(1); //noise reduction delay
          if (digitalRead(OK) == 0)
          {
            editProfileTemp(profileValueBeingEdited, profiles, currentProfile, false);
            profileEdited();
          }
        }
        else if (menuState == 4 && digitalRead(OK) == 0)
        {
          delay(1); //noise reduction delay
          if (digitalRead(OK) == 0)
          {
            editProfileTime(profileValueBeingEdited, profiles, currentProfile, false);
            profileEdited();
          }
        } else {
          menuState = 4; //We're in profile edit mode
        }
      } else {

        menuState++;
        if (menuState > 4)
          menuState = 4;
      }
    }
  }
  else if (buttonStatePLUS == 1)
  {
    if (debug) Serial.println("Plus Pressed");
    if (enableMenu == 1)
    {
      prevMenuState = menuState;
      if (currentMenuID == 5)
      {
        if (menuState == 2 && digitalRead(OK) == 0)
        {
          delay(1); //noise reduction
          if (digitalRead(OK) == 0)
          {
            profileEdited();
            editProfileTemp(profileValueBeingEdited, profiles, currentProfile, true);
          }
        }
        else if (menuState == 4 && digitalRead(OK) == 0)
        {
          delay(1); //noise reduction delay
          if (digitalRead(OK) == 0)
          {
            editProfileTime(profileValueBeingEdited, profiles, currentProfile, true);
            profileEdited();
          }
        } else {
          menuState = 2;
        }
      } else {
        menuState--;
        if (menuState < 0)
          menuState = 0;
      }
    }
  }

  if (buttonStateOK == 1)
  {
    //Use top bar as info bar and use TOP display as state progress
    //Also draw profile as we go
    if (debug) Serial.println("OK Pressed");
    if (beepState) buzzer(SPK);
    if (currentMenuID == 0)
    {
      switch (menuState)
      {
        case 0: //Config
          {
            currentMenuID = 1;
            break;
          }
        case 1: //Profile Number
          {
            currentProfile++;
            if (currentProfile > 9)
              currentProfile = 1;
            drawProfile(profiles, tc1Temp, currentProfile);
            storedVar.profile = currentProfile;
            if (previousSavedProfile != currentProfile)
            {
              writeTimeout = 10000;
              saved = false;
            } else {
              saved = true;
            }
            break;
          }
        case 2: //Profile
          {
            currentMenuID = 2;
            break;
          }
        case 3: //Tools
          {
            currentMenuID = 3;
            break;
          }
        case 4: //Start
          {
            if (backTemp < START_TEMP_MAX && frontTemp < START_TEMP_MAX)
            {
              //TODO check for thermocouple
              drawProfile(profiles, tc1Temp, currentProfile);
              u8g2.updateDisplay();

              currentMenuID = 4;

              //Update menu each second
              enableMenu = 0;
              serialPrintProfile(tcState, currentProfile, profiles, heaterPIDChan0, heaterPIDChan1, celsiusMode);

              startReflow();
            } else {
              u8g2log.print("\fERR:Temp >50");
              printError();
            }
            break;
          }
      }
    }
    else if (currentMenuID == 4) //We're in reflow
    {
      enableMenu = 1;
      currentMenuID = 0;

      if (tuning == true)
      {
        Serial.println("Tuning Cancelled");
        u8g2log.print("\fTuning Cancelled");
      } else {
        Serial.println("Reflow Cancelled");
        u8g2log.print("\fReflow Cancelled");
      }

      if (tuning == true)
      {
        tuning = false;
      }

      disableReflow();

      printError();
    }
    else if (currentMenuID == 2) //Editing profile value
    {
      currentMenuID = 5;

      //Value being edited
      profileValueBeingEdited = menuState;

      if (menuState != 2 && menuState != 4)
      {
        if (menuState < 2) { // Goto nearest valid menu item
          prevMenuState = 2;
          menuState = 2;
        } else  {
          prevMenuState = 4;
          menuState = 4;
        }
      }
    }
    else if (currentMenuID == 5) //Editing a profile value
    {
      //Handled in plus/minus button
    }
    else if (currentMenuID == 3) //Tools Menu
    {
      switch (menuState)
      {
        case 0: //Tune
          if (backTemp < START_TEMP_MAX && frontTemp < START_TEMP_MAX)
          {
            if (tuning != true) { //Set the output to the desired starting frequency.
              Serial.println("");
              Serial.println("Tuning Started");
              Serial.println("");
              Serial.println("Mode,Seconds,SetPoint,TC0,RateTC0,PWMOut0,TC1,RateTC1,PWMOut1");
              Serial.println("");
              tuning = true;
              currentState = PREHT;

              secondsMillisStartChan0 = millis();
              secondsMillisStartChan1 = millis();

              enableMenu = 0;
              currentMenuID = 4;

              startReflow();

              clearProfile();
              u8g2.updateDisplay();
            }
          } else {
            u8g2log.print("\fERR:Temp >50");
            printError();
          }
          break;
        case 1: //PID
          currentMenuID = 7;
          break;
        case 2: //Test
          currentMenuID = 6;
          break;
        case 3: //Reset
          {
            restoreDefaults(&storedVar);
            loadSettings();
            saved = false;
            writeTimeout = 1000;
          }
      }
    }
    else if (currentMenuID == 6) //Test Menu
    {
      currentState = NONE;
      switch (menuState)
      {
        case 0: //Front
          if (frontState == 0) //Turn on
          {
            frontState = 1;
          } else {
            frontState = 0;
          }
          break;
        case 1: //Back
          if (backState == 0) //Turn on
          {
            backState = 1;
          } else {
            backState = 0;
          }
          break;
        case 2: //Stir
          if (convectionState == 0) //Turn on
          {
            convectionState = 1;
          } else {
            convectionState = 0;
          }
          break;
        case 3: //Vent
          if (exhaustState == 0) //Turn on
          {
            exhaustState = 1;
          } else {
            exhaustState = 0;
          }
          break;
      }
    } else if (currentMenuID == 7) //PID menu
    {
      switch (menuState)
      {
        case 0: //Channel Selection
          currentPIDEditSelection++;
          if (currentPIDEditSelection > 5) currentPIDEditSelection = 0;
          break;
        case 1: //P
          //TODO: Be able to edit these with +/-?
          break;
        case 2: //I
          break;
        case 3: //D
          break;
      }
    } else if (currentMenuID == 1) //Config menu
    {
      switch (menuState)
      {

        case 0: //Temp units
          celsiusMode = !celsiusMode;
          valueChanged = true;
          break;
        case 1: //TC Change
          tcState++;
          if (tcState > 2) tcState = 0;
          valueChanged = true;
          break;
        case 2: //Beep State
          beepState = !beepState;
          valueChanged = true;
      }
    }

    refreshMenu();
    drawMenu(currentMenuID);
    drawMenuBox(menuState);
    u8g2.updateDisplay();
  }

  if (buttonStateBACK == buttonStateTimeout)
  {
    if (debug) Serial.println("Back Pressed");
    if (beepState) buzzer(SPK);

    switch (currentMenuID)
    {
      case 1: //Config
        currentMenuID = 0;
        if (valueChanged) //Try to save flash writes by only writing if variable actually changed when back pressed.
        {
          if (storedVar.celsiusMode != celsiusMode)
          {
            storedVar.celsiusMode = celsiusMode;
            saved = false;
            writeTimeout = 1000;
          }

          if (storedVar.tcState != tcState)
          {
            storedVar.tcState = tcState;
            saved = false;
            writeTimeout = 1000;
          }

          if (storedVar.beepState != beepState)
          {
            storedVar.beepState = beepState;
            saved = false;
            writeTimeout = 1000;
          }
        }
        valueChanged = false;
        break;
      case 2: //Profile
        currentMenuID = 0;
        if (valueChanged)
        {
          //Save Values
          for (int i = 0; i < MAX_PROFILES; i++)
          {
            storedVar.profiles[i].PreHtTemp = profiles[i + 1].PreHtTemp;
            storedVar.profiles[i].HeatTemp = profiles[i + 1].HeatTemp;
            storedVar.profiles[i].RefTemp = profiles[i + 1].RefTemp;
            storedVar.profiles[i].RefKpTemp = profiles[i + 1].RefKpTemp;
            storedVar.profiles[i].CoolTemp = profiles[i + 1].CoolTemp;

            storedVar.profiles[i].PreHtTime = profiles[i + 1].PreHtTime;
            storedVar.profiles[i].HeatTime = profiles[i + 1].HeatTime;
            storedVar.profiles[i].RefTime = profiles[i + 1].RefTime;
            storedVar.profiles[i].RefKpTime = profiles[i + 1].RefKpTime;
            storedVar.profiles[i].CoolTime = profiles[i + 1].CoolTime;
          }
          saved = false;
          writeTimeout = 1000;
        }

        valueChanged = false;
        break;
      case 3: //Tools
        currentMenuID = 0;
        break;
      case 4: //Start reflow
        enableMenu = 1;
        currentMenuID = 0;
        //Disable reflow

        if (tuning)
        {
          Serial.println("Tuning Cancelled");
          u8g2log.print("\fTuning Cancelled");
        } else {
          Serial.println("Reflow Cancelled");
          u8g2log.print("\fReflow Cancelled");
        }

        if (tuning)
        {
          tuning = false;
        }

        disableReflow();

        printError();
        break;
      case 5: //Editing a value

        currentMenuID = 2;

        drawProfile(profiles, tc1Temp, currentProfile);
        u8g2.updateDisplay();

        break;
      case 6: //Testing outputs
        currentMenuID = 3;
        /* Disable outputs */
        exhaustState = 0;
        frontState = 0;
        backState = 0;
        convectionState = 0;
        currentState = IDLEM;

        break;
      case 7:
        currentMenuID = 3;
        break;
      default:
        currentMenuID = 0;
    }
    refreshMenu();
    drawMenu(currentMenuID);
    drawMenuBox(menuState);
    u8g2.updateDisplay();
  }

  if (menuState != prevMenuState) {
    undrawMenuBox(prevMenuState);
    drawMenu(currentMenuID);
    drawMenuBox(menuState);
    u8g2.updateDisplay();
    prevMenuState = menuState;
  }

  /* Debouncing and EMI noise reduction (must be held down for timeout (4ms)) */
  if (buttonStateMINUS > 0 && digitalRead(MINUS) == 0) buttonStateMINUS--;
  if (buttonStatePLUS > 0 && digitalRead(PLUS) == 0) buttonStatePLUS--;
  if (buttonStateOK > 0 && digitalRead(OK) == 0) buttonStateOK--;
  if (buttonStateBACK > 0 && digitalRead(BACK) == 0) buttonStateBACK--;

  if (digitalRead(MINUS) == 1) buttonStateMINUS = 0;
  if (digitalRead(PLUS) == 1) buttonStatePLUS = 0;
  if (digitalRead(OK) == 1) buttonStateOK = 0;
  if (digitalRead(BACK) == 1) buttonStateBACK = 0;

  unsigned long currentTime = millis();

  if (reflow) {
    //Shouldn't overflow with the timeframes required for overflow
    currentReflowSeconds = (currentTime - reflowStartTime) / 1000;
    if ((currentReflowSeconds % 5) == 0)
    {
      previousTempDisplayUpdate = currentReflowSeconds;
      drawCurrentTemp((backTemp + frontTemp) / 2, currentReflowSeconds);
    }
  }

  if (currentTime - previousTimeReflowDisplayUpdate >= 1000 )
  {
    previousTimeReflowDisplayUpdate = currentTime;

    if (reflow) {

      refreshMenu();
      drawMenu(currentMenuID);

      u8g2.updateDisplay();

      serialPrintCurrentState(currentState);

      Serial.print(currentReflowSeconds);
      Serial.print(",");
      if (celsiusMode) Serial.print(shouldBeTemp);
      else Serial.print(cToF(shouldBeTemp));
      Serial.print(",");
      if (celsiusMode) Serial.print(backTemp);
      else Serial.print(cToF(backTemp));
      Serial.print(",");
      if (celsiusMode) Serial.print(rampRateChan0);
      else Serial.print(cToF(rampRateChan0));
      Serial.print(",");
      Serial.print(OutputChan0);
      Serial.print(",");
      if (celsiusMode) Serial.print(frontTemp);
      else Serial.print(cToF(frontTemp));
      Serial.print(",");
      if (celsiusMode) Serial.print(rampRateChan1);
      else Serial.print(cToF(rampRateChan1));
      Serial.print(",");
      Serial.println(OutputChan1);

    }
  }

  if (abs(currentTime - previousTimeReadTemp) >= pollRate) {

    previousTimeReadTemp = currentTime;

    readTemps();

    prevTempLoopMillis = currentTempLoopMillis;
    currentTempLoopMillis = millis();

    // calculate rate of temperature change (degree per second)

    rampRateChan0 = (backTemp - previousRateDisplayTemp0) * 1000 / (currentTempLoopMillis - prevTempLoopMillis);
    rampRateChan1 = (frontTemp - previousRateDisplayTemp1) * 1000 / (currentTempLoopMillis - prevTempLoopMillis);

    previousRateDisplayTemp0 = backTemp;

    previousRateDisplayTemp1 = frontTemp;

    InputChan0 = backTemp; // update the variable the PID reads

    InputChan1 = frontTemp; // update the variable the PID reads

    if (currentReflowSeconds > 1000) //16 minute safety timer
    {
      currentState = IDLEM;

      Serial.println("Reflow Safety Timer Exceeded!");
      u8g2log.print("\fReflow Cancelled");

      printError();
    }

    switch (currentState)
    {
      case PREHT:
        {
          if (currentState != previousState) { //State changed
            convectionState = 1;

            currentStateAutotuned = false;

            //if (!tuning) OutputChan0 = 80;
            OutputChan0 = 80;
            PIDChan0.SetMode(AUTOMATIC);
            PIDChan0.SetControllerDirection(DIRECT);
            PIDChan0.SetTunings(heaterPIDChan0.Kp_PREHEAT, heaterPIDChan0.Ki_PREHEAT, heaterPIDChan0.Kd_PREHEAT);
            SetpointChan0 = backTemp;

            //if (!tuning) OutputChan1 = 80;
            OutputChan1 = 80;
            PIDChan1.SetMode(AUTOMATIC);
            PIDChan1.SetControllerDirection(DIRECT);
            PIDChan1.SetTunings(heaterPIDChan1.Kp_PREHEAT, heaterPIDChan1.Ki_PREHEAT, heaterPIDChan1.Kd_PREHEAT);
            SetpointChan1 = frontTemp;

            startTemp = (backTemp + frontTemp) / 2;
            startStateMillis = currentTime;

            previousRateDisplayTemp0 = 0;

            previousRateDisplayTemp1 = 0;

            if (tuning)
            {
              // Set the target value to tune to
              // This will depend on what you are tuning. This should be set to a value within
              // the usual range of the setpoint. For low-inertia systems, values at the lower
              // end of this range usually give better results. For anything else, start with a
              // value at the middle of the range.
              tunerChan0.setTargetInputValue(SetpointChan0);
              tunerChan1.setTargetInputValue(SetpointChan1);

              // Set the loop interval in microseconds
              // This must be the same as the interval the PID control loop will run at
              tunerChan0.setLoopInterval(((unsigned long)pollRate) * 1000); //microseconds
              tunerChan1.setLoopInterval(((unsigned long)pollRate) * 1000); //microseconds

              // Set the output range
              tunerChan0.setOutputRange(0, 100);
              tunerChan1.setOutputRange(0, 100);

              // Set the Ziegler-Nichols tuning mode
              // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
              // or PIDAutotuner::ZNModeNoOvershoot. Test with ZNModeBasicPID first, but if there
              // is too much overshoot you can try the others.
              tunerChan0.setZNMode(PIDAutotuner::ZNModeBasicPID);
              tunerChan1.setZNMode(PIDAutotuner::ZNModeBasicPID);

              // This must be called immediately before the tuning loop
              tunerChan0.startTuningLoop();
              tunerChan1.startTuningLoop();
            }

          }
          previousState = currentState;

          double rate = ((double)profiles[currentProfile].PreHtTemp - startTemp) / (double)profiles[currentProfile].PreHtTime;

          if (tuning) {
            shouldBeTemp = profiles[currentProfile].PreHtTemp;
          } else {
            shouldBeTemp = ((double)abs(currentTime - startStateMillis)) / 1000 * rate + startTemp;
          }

          /* If we don't reach the temp in time, hold the temp as setpoint */
          if (shouldBeTemp > profiles[currentProfile].PreHtTemp)// && tc0Temp < profiles[currentProfile].PreHtTemp && tc1Temp < profiles[currentProfile].PreHtTemp)
          {
            shouldBeTemp = profiles[currentProfile].PreHtTemp;
            SetpointChan0 = shouldBeTemp + 5; //"help" the PID system overshoot in these early reflow stages to speed up transition
            SetpointChan1 = shouldBeTemp + 5;
          } else {

            SetpointChan0 = shouldBeTemp;
            SetpointChan1 = shouldBeTemp;
          }

          if (tuning) {

            tunerChan0.setTargetInputValue(shouldBeTemp);
            tunerChan1.setTargetInputValue(shouldBeTemp);

            OutputChan0 = tunerChan0.tunePID(InputChan0);
            OutputChan1 = tunerChan1.tunePID(InputChan1);

            if (tunerChan0.isFinished() && tunerChan1.isFinished() && !currentStateAutotuned) {
              currentStateAutotuned = true;
              currentState = HEAT;

              heaterPIDChan0.Kp_PREHEAT = tunerChan0.getKp();
              heaterPIDChan0.Ki_PREHEAT = tunerChan0.getKi();
              heaterPIDChan0.Kd_PREHEAT = tunerChan0.getKd();

              heaterPIDChan1.Kp_PREHEAT = tunerChan1.getKp();
              heaterPIDChan1.Ki_PREHEAT = tunerChan1.getKi();
              heaterPIDChan1.Kd_PREHEAT = tunerChan1.getKd();

              storedVar.KpChan0_PREHEAT = heaterPIDChan0.Kp_PREHEAT;
              storedVar.KiChan0_PREHEAT = heaterPIDChan0.Ki_PREHEAT;
              storedVar.KdChan0_PREHEAT = heaterPIDChan0.Kd_PREHEAT;

              storedVar.KpChan1_PREHEAT = heaterPIDChan1.Kp_PREHEAT;
              storedVar.KiChan1_PREHEAT = heaterPIDChan1.Ki_PREHEAT;
              storedVar.KdChan1_PREHEAT = heaterPIDChan1.Kd_PREHEAT;

              Serial.println("Preheat Tune Done");
              u8g2log.print("\fPreheat Tune Done");
              serialPrintTuningDonePreheat(heaterPIDChan0, heaterPIDChan1);

              printError();

            }
          } else {
            if (backTemp > profiles[currentProfile].PreHtTemp - 1 && frontTemp > profiles[currentProfile].PreHtTemp - 1) {
              currentState = HEAT;
            }
          }

          break;
        }

      case HEAT:
        {
          if (currentState != previousState) { //State changed
            convectionState = 1;

            currentStateAutotuned = false;

            SetpointChan0 = backTemp;

            SetpointChan1 = frontTemp;

            PIDChan0.SetTunings(heaterPIDChan0.Kp_SOAK, heaterPIDChan0.Ki_SOAK, heaterPIDChan0.Kd_SOAK);
            PIDChan1.SetTunings(heaterPIDChan1.Kp_SOAK, heaterPIDChan1.Ki_SOAK, heaterPIDChan1.Kd_SOAK);

            if (tuning)
            {
              tunerChan0.setTargetInputValue(profiles[currentProfile].HeatTemp);
              tunerChan1.setTargetInputValue(profiles[currentProfile].HeatTemp);

              tunerChan0.startTuningLoop();
              tunerChan1.startTuningLoop();
            }

            startTemp = (backTemp + frontTemp) / 2;
            startStateMillis = currentTime;

          }
          previousState = currentState;

          double rate = ((double)profiles[currentProfile].HeatTemp - startTemp) / (double)profiles[currentProfile].HeatTime;

          if (tuning) {
            shouldBeTemp = profiles[currentProfile].HeatTemp;
          } else {

            shouldBeTemp = ((double)abs(currentTime - startStateMillis)) / 1000 * rate + profiles[currentProfile].PreHtTemp;
          }

          /* If we don't reach the temp in time, hold the temp as setpoint */
          if (shouldBeTemp > profiles[currentProfile].HeatTemp)// && tc0Temp < profiles[currentProfile].HeatTemp && tc1Temp < profiles[currentProfile].HeatTemp)
          {
            shouldBeTemp = profiles[currentProfile].HeatTemp;
            SetpointChan0 = shouldBeTemp + 5; //"help" the PID system overshoot in these early reflow stages to speed up transition
            SetpointChan1 = shouldBeTemp + 5;
          } else {

            SetpointChan0 = shouldBeTemp;
            SetpointChan1 = shouldBeTemp;
          }
          if (tuning) {

            tunerChan0.setTargetInputValue(shouldBeTemp);
            tunerChan1.setTargetInputValue(shouldBeTemp);

            OutputChan0 = tunerChan0.tunePID(InputChan0);
            OutputChan1 = tunerChan1.tunePID(InputChan1);

            if (tunerChan0.isFinished() && tunerChan1.isFinished() && !currentStateAutotuned) {

              tunerChan0.startTuningLoop();
              tunerChan1.startTuningLoop();

              currentStateAutotuned = true;
              currentState = REF;

              heaterPIDChan0.Kp_SOAK = tunerChan0.getKp();
              heaterPIDChan0.Ki_SOAK = tunerChan0.getKi();
              heaterPIDChan0.Kd_SOAK = tunerChan0.getKd();

              heaterPIDChan1.Kp_SOAK = tunerChan1.getKp();
              heaterPIDChan1.Ki_SOAK = tunerChan1.getKi();
              heaterPIDChan1.Kd_SOAK = tunerChan1.getKd();

              storedVar.KpChan0_SOAK = heaterPIDChan0.Kp_SOAK;
              storedVar.KiChan0_SOAK = heaterPIDChan0.Ki_SOAK;
              storedVar.KdChan0_SOAK = heaterPIDChan0.Kd_SOAK;

              storedVar.KpChan1_SOAK = heaterPIDChan1.Kp_SOAK;
              storedVar.KiChan1_SOAK = heaterPIDChan1.Ki_SOAK;
              storedVar.KdChan1_SOAK = heaterPIDChan1.Kd_SOAK;

              Serial.println("Heat Tune Done");
              u8g2log.print("\fHeat Tune Done");

              serialPrintTuningDoneHeat(heaterPIDChan0, heaterPIDChan1);

              printError();

            }
          } else {

            if (backTemp >= profiles[currentProfile].HeatTemp - 1 && frontTemp >= profiles[currentProfile].HeatTemp - 1) {
              currentState = REF;
            }
          }
          break;
        }

      case REF:
        {
          if (currentState != previousState) { //State changed
            convectionState = 1;

            SetpointChan0 = backTemp;

            SetpointChan1 = frontTemp;

            PIDChan0.SetTunings(heaterPIDChan0.Kp_REFLOW, heaterPIDChan0.Ki_REFLOW, heaterPIDChan0.Kd_REFLOW);
            PIDChan1.SetTunings(heaterPIDChan1.Kp_REFLOW, heaterPIDChan1.Ki_REFLOW, heaterPIDChan1.Kd_REFLOW);

            if (tuning)
            {
              tunerChan0.setTargetInputValue(profiles[currentProfile].RefTemp);
              tunerChan1.setTargetInputValue(profiles[currentProfile].RefTemp);

              tunerChan0.startTuningLoop();
              tunerChan1.startTuningLoop();
            }

            startTemp = (backTemp + frontTemp) / 2;
            startStateMillis = currentTime;

          }
          previousState = currentState;

          double rate = ((double)profiles[currentProfile].RefTemp - startTemp) / (double)profiles[currentProfile].RefTime;

          if (tuning) {
            shouldBeTemp = profiles[currentProfile].RefTemp;
          } else {
            shouldBeTemp = ((double)abs(currentTime - startStateMillis)) / 1000 * rate + profiles[currentProfile].HeatTemp;
          }

          /* If we don't reach the temp in time, hold the temp as setpoint */
          if (shouldBeTemp > profiles[currentProfile].RefTemp)// && tc0Temp < profiles[currentProfile].RefTemp && tc1Temp < profiles[currentProfile].RefTemp)
          {
            shouldBeTemp = profiles[currentProfile].RefTemp;
            SetpointChan0 = shouldBeTemp;
            SetpointChan1 = shouldBeTemp;
          } else {

            SetpointChan0 = shouldBeTemp;
            SetpointChan1 = shouldBeTemp;
          }

          if (tuning) {
            tunerChan0.setTargetInputValue(shouldBeTemp);
            tunerChan1.setTargetInputValue(shouldBeTemp);

            OutputChan0 = tunerChan0.tunePID(InputChan0);
            OutputChan1 = tunerChan1.tunePID(InputChan1);

            if (tunerChan0.isFinished() && tunerChan1.isFinished()) {
              currentState = IDLEM; //Go straight to idle as we aren't going to tune refkp or cool

              enableMenu = 1;
              currentMenuID = 0;
              disableReflow();
              tuning = false;

              heaterPIDChan0.Kp_REFLOW = tunerChan0.getKp();
              heaterPIDChan0.Ki_REFLOW = tunerChan0.getKi();
              heaterPIDChan0.Kd_REFLOW = tunerChan0.getKd();

              heaterPIDChan1.Kp_REFLOW = tunerChan1.getKp();
              heaterPIDChan1.Ki_REFLOW = tunerChan1.getKi();
              heaterPIDChan1.Kd_REFLOW = tunerChan1.getKd();

              storedVar.KpChan0_REFLOW = heaterPIDChan0.Kp_REFLOW;
              storedVar.KiChan0_REFLOW = heaterPIDChan0.Ki_REFLOW;
              storedVar.KdChan0_REFLOW = heaterPIDChan0.Kd_REFLOW;

              storedVar.KpChan1_REFLOW = heaterPIDChan1.Kp_REFLOW;
              storedVar.KiChan1_REFLOW = heaterPIDChan1.Ki_REFLOW;
              storedVar.KdChan1_REFLOW = heaterPIDChan1.Kd_REFLOW;

              Serial.println("Reflow Tune Done");
              u8g2log.print("\fReflow Tune Done");

              serialPrintTuningDoneReflow(heaterPIDChan0, heaterPIDChan1);

              printError();

              delay(1000);

              Serial.println("Tuning Complete");
              u8g2log.print("\fTuning Complete");

              printError();
              writeTimeout = 1000;
              saved = false;

              refreshMenu();
              drawMenu(currentMenuID);
              drawMenuBox(menuState);
              u8g2.updateDisplay();

            }

          } else {

            if (backTemp >= profiles[currentProfile].RefTemp - 2 && frontTemp >= profiles[currentProfile].RefTemp - 2 ) {
              currentState = REFKP;
            }
          }

          break;
        }
      case REFKP:
        {
          if (currentState != previousState) { //State changed
            convectionState = 1;

            SetpointChan0 = profiles[currentProfile].RefKpTemp;

            SetpointChan1 = profiles[currentProfile].RefKpTemp;

            PIDChan0.SetTunings(heaterPIDChan0.Kp_REFLOW, heaterPIDChan0.Ki_REFLOW, heaterPIDChan0.Kd_REFLOW);
            PIDChan1.SetTunings(heaterPIDChan1.Kp_REFLOW, heaterPIDChan1.Ki_REFLOW, heaterPIDChan1.Kd_REFLOW);

            startTemp = (backTemp + frontTemp) / 2;
            startStateMillis = currentTime;

          }
          previousState = currentState;

          if (abs((currentTime - startStateMillis) / 1000) >= (unsigned int)profiles[currentProfile].RefKpTime) {
            currentState = COOL;
          }
          break;
        }

      case COOL:
        {
          if (currentState != previousState) { //State changed
            convectionState = 1;

            PIDChan0.SetControllerDirection(REVERSE);
            PIDChan0.SetTunings(fanPID.Kp, fanPID.Ki, fanPID.Kd);
            SetpointChan0 = backTemp;

            PIDChan1.SetControllerDirection(REVERSE);
            PIDChan1.SetTunings(fanPID.Kp, fanPID.Ki, fanPID.Kd);
            SetpointChan1 = frontTemp;

            //Start fan up full so it doesn't have to catch up with PID on initial state change
            OutputChan0 = 100;
            OutputChan1 = 100;

            startTemp = (backTemp + frontTemp) / 2;

            startStateMillis = currentTime;
          }
          previousState = currentState;

          double rate = (startTemp - (double)profiles[currentProfile].CoolTemp) / (double)profiles[currentProfile].CoolTime;

          shouldBeTemp = (double)profiles[currentProfile].RefKpTemp - (((double)abs(currentTime - startStateMillis)) / 1000 * rate);

          /* If we don't reach the temp in time, hold the temp as setpoint */
          if (shouldBeTemp < profiles[currentProfile].CoolTemp)
          {
            shouldBeTemp = profiles[currentProfile].CoolTemp;
          }

          SetpointChan0 = shouldBeTemp;
          SetpointChan1 = shouldBeTemp;

          if (backTemp <= profiles[currentProfile].CoolTemp + 2 || frontTemp <= profiles[currentProfile].CoolTemp + 2 ) {
            PIDChan0.SetMode(MANUAL);
            PIDChan1.SetMode(MANUAL);
            currentState = IDLEM;
            enableMenu = 1;
            currentMenuID = 0;
            disableReflow();
            Serial.println("Reflow Complete");
            u8g2log.print("\fReflow Complete");
            printError();

            refreshMenu();
            drawMenu(currentMenuID);
            drawMenuBox(menuState);
            u8g2.updateDisplay();
          }

          break;
        }
      case IDLEM:
        {
          if (currentState != previousState) { //State changed
            PIDChan0.SetMode(MANUAL);
            PIDChan1.SetMode(MANUAL);

          }
          previousState = currentState;
          convectionState = 0;

          //If temp over 50 and IDLE, enable fan

          if (backTemp > START_TEMP_MAX || frontTemp > START_TEMP_MAX)
          {
            OutputChan0 = 100;
            OutputChan1 = 100;
          } else if (backTemp < START_TEMP_MAX - 10 && frontTemp < START_TEMP_MAX - 10) { //10 degree hysteresis
            OutputChan0 = 0;
            OutputChan1 = 0;
          }
          break;
        }
      default:
      case NONE:
        {
          break;
        }
    }
    if (currentState != IDLEM && tuning == false)
    {
      PIDChan0.Compute();
      PIDChan1.Compute();
    }
  }

  if (OutputChan0 > 100) OutputChan0 = 100;
  if (OutputChan1 > 100) OutputChan1 = 100;
  if (OutputChan0 < 0) OutputChan0 = 0;
  if (OutputChan1 < 0) OutputChan1 = 0;

  if (currentState != NONE) // no PWM in NONE
  {
    //Create rough "PWM" (second long)
    if (currentState != COOL && currentState != IDLEM) {
      exhaustState = 0;
      if (currentTime - secondsMillisStartChan0 <= OutputChan0 * 10)
      {
        //Turn on
        backState = 1;
      } else {
        if (currentTime - secondsMillisStartChan0 > 1000) secondsMillisStartChan0 = millis(); //Reset PWM
        if (OutputChan0 < 100) backState = 0;
      }

      if (currentTime - secondsMillisStartChan1 <= OutputChan1 * 10)
      {
        frontState = 1;
      } else {
        if (currentTime - secondsMillisStartChan1 > 1000) secondsMillisStartChan1 = millis(); //Reset PWM
        if (OutputChan1 < 100) frontState = 0;
      }

    } else {

      backState = 0;
      frontState = 0;

      if (currentTime - secondsMillisStartChan0 <= OutputChan0 * 10 || currentTime - secondsMillisStartChan1 <= OutputChan1 * 10)
      {
        //Turn on fan
        exhaustState = 1;
      } else {
        if (currentTime - secondsMillisStartChan0 > 1000) secondsMillisStartChan0 = millis(); //Reset PWM
        if (currentTime - secondsMillisStartChan1 > 1000) secondsMillisStartChan1 = millis(); //Reset PWM
        if (OutputChan0 < 100 || OutputChan1 < 100) exhaustState = 0;
      }
    }
  }

  if (writeTimeout == 0 && saved == false)
  {
    previousSavedProfile = currentProfile;
    saved = true;
    storedVarFlash.write(storedVar);
    //Serial.println("settings stored");
    u8g2log.print("\fSettings Stored");
    printError();
  }

  if (writeTimeout >= 0) writeTimeout--;

  if (currentTime - previousTimeUpdateTemp >= 2000) {

    previousTimeUpdateTemp = currentTime;
    updateTemps();
  }

  checkSerial();
  delay(1);
}
