#include "MAX31855soft.h" //https://github.com/enjoyneering/MAX31855
//#include <openGLCD.h>
//#include <U8x8lib.h>
#include <U8g2lib.h>
#include "splash.h"
#include "helper.h"
#include "FlashStorage.h"
#include "PID_v1.h"
#include "pidautotuner.h"

//ReflowController project used as source of inspiration for PID control
//https://github.com/0xPIT/reflowOvenController/blob/master/ReflowController/ReflowController.ino
//

// ----------------------------------------------------------------------------
// PID

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


// ******************* DEFAULT PID PARAMETERS *******************
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT_C0 40 //50.28 //
#define PID_KI_PREHEAT_C0 0.025 //4.78 //
#define PID_KD_PREHEAT_C0 20 //166.72 //

#define PID_KP_PREHEAT_C1 40 //96.98 //
#define PID_KI_PREHEAT_C1 0.025 //26.96 //
#define PID_KD_PREHEAT_C1 20 //210.38 //

// ***** SOAKING STAGE *****
#define PID_KP_SOAK_C0 200 //97.00 //
#define PID_KI_SOAK_C0 0.015 //25.97 //
#define PID_KD_SOAK_C0 50 //168.54 //

#define PID_KP_SOAK_C1 200 //44.30 //
#define PID_KI_SOAK_C1 0.015 //6.56 //
#define PID_KD_SOAK_C1 50 //201.32 //

// ***** REFLOW STAGE *****
#define PID_KP_REFLOW_C0 100 //38.12 //
#define PID_KI_REFLOW_C0 0.025 //2.15 //
#define PID_KD_REFLOW_C0 25 //200.13 //

#define PID_KP_REFLOW_C1 100 //43.01 //
#define PID_KI_REFLOW_C1 0.025 //6.73 //
#define PID_KD_REFLOW_C1 25 //400.31 //

typedef struct {
  double Kp_PREHEAT;
  double Ki_PREHEAT;
  double Kd_PREHEAT;

  double Kp_SOAK;
  double Ki_SOAK;
  double Kd_SOAK;

  double Kp_REFLOW;
  double Ki_REFLOW;
  double Kd_REFLOW;
} PID3_t;

typedef struct {
  double Kp;
  double Ki;
  double Kd;
} PID_t;

PID3_t heaterPIDChan0 = { PID_KP_PREHEAT_C0, PID_KI_PREHEAT_C0, PID_KD_PREHEAT_C0, PID_KP_SOAK_C0, PID_KI_SOAK_C0, PID_KD_SOAK_C0, PID_KP_REFLOW_C0, PID_KI_REFLOW_C0, PID_KD_REFLOW_C0 };
PID3_t heaterPIDChan1 = { PID_KP_PREHEAT_C1, PID_KI_PREHEAT_C1, PID_KD_PREHEAT_C1, PID_KP_SOAK_C1, PID_KI_SOAK_C1, PID_KD_SOAK_C1, PID_KP_REFLOW_C1, PID_KI_REFLOW_C1, PID_KD_REFLOW_C1 };
PID_t fanPID    = { 10.00, 0.03, 10.00 }; //{ 1.00, 0.03, 10.00 };

PID PIDChan0(&InputChan0, &OutputChan0, &SetpointChan0, heaterPIDChan0.Kp_PREHEAT, heaterPIDChan0.Ki_PREHEAT, heaterPIDChan0.Kd_PREHEAT, DIRECT);

/*PID_ATune PIDTuneChan0(&InputChan0, &OutputChan0);

  double aTuneStepChan0       =  50,
       aTuneNoiseChan0      =   1,
       aTuneStartValueChan0 =  50; // is set to Output, i.e. 0-100% of Heater //90 will get set to 100 because of stepChan

  unsigned int aTuneLookBackChan0 = 10; //Seconds*/

PIDAutotuner tunerChan0 = PIDAutotuner();

PID PIDChan1(&InputChan1, &OutputChan1, &SetpointChan1, heaterPIDChan1.Kp_PREHEAT, heaterPIDChan1.Ki_PREHEAT, heaterPIDChan1.Kd_PREHEAT, DIRECT);

/*PID_ATune PIDTuneChan1(&InputChan1, &OutputChan1);

  double aTuneStepChan1       =  50,
       aTuneNoiseChan1      =   1,
       aTuneStartValueChan1 =  50; // is set to Output, i.e. 0-100% of Heater

  unsigned int aTuneLookBackChan1 = 10; //Seconds*/

PIDAutotuner tunerChan1 = PIDAutotuner();

#define MAX_PROFILES 9

int currentState = IDLEM;

double rampRateChan0 = 0;
double rampRateChan1 = 0;

U8G2_KS0108_128X64_F u8g2(U8G2_R0, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, /*enable=*/E, /*dc=*/DI, /*cs0=*/CS1, /*cs1=*/CS2, /*cs2=*/U8X8_PIN_NONE, /* reset=*/RST);
U8G2LOG u8g2log;

struct StoredVar
{
  bool valid;                //Should be false after new code write (to pull defaults)

  double KpChan0_PREHEAT;
  double KiChan0_PREHEAT;
  double KdChan0_PREHEAT;
  double KpChan1_PREHEAT;
  double KiChan1_PREHEAT;
  double KdChan1_PREHEAT;

  double KpChan0_SOAK;
  double KiChan0_SOAK;
  double KdChan0_SOAK;
  double KpChan1_SOAK;
  double KiChan1_SOAK;
  double KdChan1_SOAK;

  double KpChan0_REFLOW;
  double KiChan0_REFLOW;
  double KdChan0_REFLOW;
  double KpChan1_REFLOW;
  double KiChan1_REFLOW;
  double KdChan1_REFLOW;

  bool celsiusMode;

  uint8_t profile;
  ReflowProfile profiles[MAX_PROFILES];
} storedVar;

bool saved = true;

int errorTimeout = 0;

void restoreDefaults()
{
  storedVar.profile = 1;

  storedVar.KpChan0_PREHEAT = PID_KP_PREHEAT_C0;
  storedVar.KiChan0_PREHEAT = PID_KI_PREHEAT_C0;
  storedVar.KdChan0_PREHEAT = PID_KD_PREHEAT_C0;
  storedVar.KpChan1_PREHEAT = PID_KP_PREHEAT_C1;
  storedVar.KiChan1_PREHEAT = PID_KI_PREHEAT_C1;
  storedVar.KdChan1_PREHEAT = PID_KD_PREHEAT_C1;

  storedVar.KpChan0_SOAK = PID_KP_SOAK_C0;
  storedVar.KiChan0_SOAK = PID_KI_SOAK_C0;
  storedVar.KdChan0_SOAK = PID_KD_SOAK_C0;
  storedVar.KpChan1_SOAK = PID_KP_SOAK_C1;
  storedVar.KiChan1_SOAK = PID_KI_SOAK_C1;
  storedVar.KdChan1_SOAK = PID_KD_SOAK_C1;

  storedVar.KpChan0_REFLOW = PID_KP_REFLOW_C0;
  storedVar.KiChan0_REFLOW = PID_KI_REFLOW_C0;
  storedVar.KdChan0_REFLOW = PID_KD_REFLOW_C0;
  storedVar.KpChan1_REFLOW = PID_KP_REFLOW_C1;
  storedVar.KiChan1_REFLOW = PID_KI_REFLOW_C1;
  storedVar.KdChan1_REFLOW = PID_KD_REFLOW_C1;
  storedVar.valid = true;
  storedVar.celsiusMode = true;

  for (int i = 0; i < MAX_PROFILES; i++)
  {

    storedVar.profiles[i].PreHtTemp = 100;
    storedVar.profiles[i].HeatTemp = 180;
    storedVar.profiles[i].RefTemp = 240;
    storedVar.profiles[i].RefKpTemp = 240;
    storedVar.profiles[i].CoolTemp = 60;

    storedVar.profiles[i].PreHtTime = 80;
    storedVar.profiles[i].HeatTime = 180;
    storedVar.profiles[i].RefTime = 60;
    storedVar.profiles[i].RefKpTime = 30;
    storedVar.profiles[i].CoolTime = 80;
  }
}

int writeTimeout = -1;

FlashStorage(storedVarFlash, StoredVar);

// assume 4x6 font, define width and height
#define U8LOG_WIDTH 32
#define U8LOG_HEIGHT 1

volatile int frontState = 0;
volatile int backState = 0;
volatile int convectionState = 0;
volatile int exhaustState = 0;

// allocate memory
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];

#define BOOT_MAGIC_ADDRESS (0x20007FFCul)
#define BOOT_MAGIC_VALUE (*((volatile uint32_t *)BOOT_MAGIC_ADDRESS))
#define BOOT_MAGIC_BOOTLOADER_ENABLE 0x32

MAX31855soft MAX31855_0(CSU2, SO, SCK);
MAX31855soft MAX31855_1(CSU3, SO, SCK);

#define NUMBER_OF_TEMP_AVERAGES 6

double tc0History[NUMBER_OF_TEMP_AVERAGES] = {0};
double tc1History[NUMBER_OF_TEMP_AVERAGES] = {0};

int averagesCount = 0;

int tempHistoryHead = 0;

double tc0Temp; //Back
double tc1Temp; //Front

double tc0PrevTemp;
double tc1PrevTemp;

int tc0Detect = MAX31855_THERMOCOUPLE_OK;
int tc1Detect = MAX31855_THERMOCOUPLE_OK;

int currentProfile = 1;

#define START_TEMP_MAX 50

ReflowProfile profiles[MAX_PROFILES + 1]; //Don't use 0 profile

double boardTemp;

void drawScreen();
void drawMenu(int index);
void drawMenuBox(int index);
void drawProfile();

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
double cToF(double celsius)
{
  return ((double)1.8 * celsius) + 32;
}

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

bool profileValueChanged = false;
int profileValueBeingEdited = 0;

int previousSavedProfile;


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

#define NUMBER_OF_CROSSINGS_AVERAGES 2

volatile unsigned long previousCrossingTime;
volatile unsigned long crossings[NUMBER_OF_CROSSINGS_AVERAGES] = {0};
volatile int crossingsHead = 0;
volatile double crossingAverage;

void zeroCrossing()
{

  unsigned long currentCrossingTime = micros();

  if (frontState == 1) //Turn on
  {
    digitalWrite(FRONT_HEATER, 0);
  } else { //Turn off
    digitalWrite(FRONT_HEATER, 1);
  }
  if (backState == 1) //Turn on
  {
    digitalWrite(BACK_HEATER, 0);
  } else { //Turn off
    digitalWrite(BACK_HEATER, 1);
  }
  if (convectionState == 1) //Turn on
  {
    digitalWrite(CONVECTION, 0);
  } else { //Turn off
    digitalWrite(CONVECTION, 1);
  }
  if (exhaustState == 1) //Turn on
  {
    digitalWrite(EXHAUST, 0);
  } else { //Turn off
    digitalWrite(EXHAUST, 1);
  }

  long long crossingDelta = (long long)currentCrossingTime - (long long)previousCrossingTime;

  if (crossingDelta > 0) //Check if we had an overflow
  {
    crossings[crossingsHead] = (unsigned long)crossingDelta;

    previousCrossingTime = currentCrossingTime;

    crossingAverage = avg(crossings, NUMBER_OF_CROSSINGS_AVERAGES);

    crossingsHead++;
    if (crossingsHead >= NUMBER_OF_CROSSINGS_AVERAGES)
      crossingsHead = 0;
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

  celsiusMode = storedVar.celsiusMode;

  PIDChan0.SetTunings(heaterPIDChan0.Kp_PREHEAT, heaterPIDChan0.Ki_PREHEAT, heaterPIDChan0.Kd_PREHEAT);
  PIDChan1.SetTunings(heaterPIDChan1.Kp_PREHEAT, heaterPIDChan1.Ki_PREHEAT, heaterPIDChan1.Kd_PREHEAT);

  currentProfile = storedVar.profile;
  previousSavedProfile = currentProfile; //Used to reduce writes to flash on a common menu function
}

void checkBootloader()
{
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == '#')
    {
      /* Set bootloader check value and reboot */
      BOOT_MAGIC_VALUE = BOOT_MAGIC_BOOTLOADER_ENABLE;
      NVIC_SystemReset();//software reset
    }/* else if (c == 'R') //Command to reset for debug purposes
    {
      BOOT_MAGIC_VALUE = 0;
      NVIC_SystemReset();//software reset
    }*/
  }
}

void setup()
{
  //Protect against bad code requiring ICSP flashing, do not remove
  //This allows us to hold R on startup if we had a bad flash
  Serial.begin(115200);
  int countdown = 1000;
  while (countdown > 0)
  {
    checkBootloader();
    countdown--;
    delay(1);
  }
  //Keep above here

  //System is too noisy for this to work.

  /* Detect existing board running for 0.5s */
  /*
    attachInterrupt(digitalPinToInterrupt(DB5), otherRunningInterruptDB5, FALLING);
    attachInterrupt(digitalPinToInterrupt(DB6), otherRunningInterruptDB6, FALLING);

    int timeout = 20;
    while (timeout > 0)
    {
    delay(10);
    timeout--;
    }

    //if other board is running, go into loop
    while (otherRunning == true) {}

    detachInterrupt(digitalPinToInterrupt(DB5));
    detachInterrupt(digitalPinToInterrupt(DB6));*/

  pinMode(PLUS, INPUT);
  pinMode(MINUS, INPUT);
  pinMode(OK, INPUT);
  pinMode(BACK, INPUT);

  pinMode(RW, OUTPUT);
  digitalWrite(RW, 0);

  pinMode(FRONT_HEATER, OUTPUT);
  pinMode(BACK_HEATER, OUTPUT);
  pinMode(CONVECTION, OUTPUT);
  pinMode(EXHAUST, OUTPUT);

  /* Disable outputs */
  digitalWrite(FRONT_HEATER, 1);
  digitalWrite(BACK_HEATER, 1);
  digitalWrite(CONVECTION, 1);
  digitalWrite(EXHAUST, 1);

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

  //BACK button not on interrupt

  attachInterrupt(digitalPinToInterrupt(ZER_D), zeroCrossing, RISING);

  /* start MAX31855 */
  MAX31855_0.begin();
  MAX31855_1.begin();

  /* Get EEPROM Settings */
  storedVar = storedVarFlash.read();
  if (storedVar.valid == false) //Should be false after new code write
  {
    restoreDefaults();
  }

  loadSettings();

  PIDChan0.SetOutputLimits(0, 100); // max output 100%
  PIDChan0.SetMode(AUTOMATIC);

  PIDChan1.SetOutputLimits(0, 100); // max output 100%
  PIDChan1.SetMode(AUTOMATIC);

  delay(2000);

  u8g2.clearBuffer();

  drawScreen();
  drawMenu(0); //Draw main menu
  drawMenuBox(0);
  drawProfile();

  u8g2.sendBuffer();
}


//This translates from normal cartesian lines to LCD library x,y lines inside profile draw area
void profileBoxLine(int x1, int y1, int x2, int y2)
{
  //start y 7, size 56
  //start x 1, size 92
  //u8g2.drawFrame(1, 7, 92, 56);

  u8g2.drawLine(x1 + 1, 56 - y1 + 7, x2 + 1, 56 - y2 + 7);
}

void profileLine(int startTemp, int endTemp, int startTime, int endTime)
{
  //Temp divided into 14x4 ticks
  //Max temp 280oC (each tick is 20 degrees or 5 degrees per pixel)

  //Time divided into 23x4 ticks
  //Max time around 7.6 minutes (each tick is 20 seconds or 5 seconds per pixel)

  double y1 = ((double)startTemp) / 280 * 56;
  double y2 = ((double)endTemp) / 280 * 56;

  double x1 = ((double)startTime) / 460 * 92;
  double x2 = ((double)endTime) / 460 * 92;

  profileBoxLine(x1, y1, x2, y2);
}

//This translates from normal cartesian lines to LCD library x,y pixels inside profile draw area
void profileBoxPixel(int x, int y)
{
  //start y 7, size 56
  //start x 1, size 92
  //u8g2.drawFrame(1, 7, 92, 56);

  u8g2.drawPixel(x + 1, 56 - y + 7);
}

void drawCurrentTemp(int currentTemp, int currentTime)
{
  //Temp divided into 14x4 ticks
  //Max temp 280oC (each tick is 20 degrees or 5 degrees per pixel)

  //Time divided into 23x4 ticks
  //Max time around 7.6 minutes (each tick is 20 seconds or 5 seconds per pixel)
  if (currentTemp < 280 && currentTemp > 0 && currentTime > 0)
  {
    double y = ((double)currentTemp) / 280 * 56;

    double x = ((double)(currentTime % 460)) / 460 * 92; //Wrap around

    profileBoxPixel(x, y);
  }
}

void clearProfile()
{
  //Clear box
  u8g2.setDrawColor(0);
  u8g2.drawBox(1, 7, 92, 56);
  u8g2.setDrawColor(1);

  // Draw Ticks (Temp)
  for (int i = 6 + 4; i < 64; i = i + 4)
  {
    u8g2.drawPixel(1, i);
  }

  // Draw Ticks (Time)
  for (int i = 1; i < 90; i = i + 4)
  {
    u8g2.drawPixel(i, 62);
  }
}

void drawProfile()
{
  clearProfile();

  if (tc1Temp > 0)
  {
    profileLine(tc1Temp, profiles[currentProfile].PreHtTemp, 0, profiles[currentProfile].PreHtTime);
  } else {
    profileLine(0, profiles[currentProfile].PreHtTemp, 0, profiles[currentProfile].PreHtTime);
  }
  unsigned long tempStart = 0;
  profileLine(tc1Temp, profiles[currentProfile].PreHtTemp, tempStart, tempStart + profiles[currentProfile].PreHtTime);
  tempStart += profiles[currentProfile].PreHtTime;
  profileLine(profiles[currentProfile].PreHtTemp, profiles[currentProfile].HeatTemp, tempStart, tempStart + profiles[currentProfile].HeatTime);
  tempStart += profiles[currentProfile].HeatTime;
  profileLine(profiles[currentProfile].HeatTemp, profiles[currentProfile].RefTemp, tempStart, tempStart + profiles[currentProfile].RefTime);
  tempStart += profiles[currentProfile].RefTime;
  profileLine(profiles[currentProfile].RefTemp, profiles[currentProfile].RefKpTemp, tempStart, tempStart + profiles[currentProfile].RefKpTime);
  tempStart += profiles[currentProfile].RefKpTime;
  profileLine(profiles[currentProfile].RefKpTemp, profiles[currentProfile].CoolTemp, tempStart, tempStart + profiles[currentProfile].CoolTime);
}

int currentPIDEditSelection = 0;

void drawMenu(int index)
{
  switch (index)
  {
    default: //Goto main menu
    case 0: //Main Menu
      u8g2.setCursor(99, 14);
      u8g2.print("Config");
      u8g2.setCursor(99, 24);
      u8g2.print("Prof ");
      u8g2.print(currentProfile);
      u8g2.setCursor(97, 34);
      u8g2.print("Profile");
      u8g2.setCursor(101, 44);
      u8g2.print("Tools");
      u8g2.setCursor(101, 57);
      u8g2.print("Start");
      break;
    case 1: //Config Menu
      u8g2.setCursor(107, 14);
      if (celsiusMode) u8g2.print("oC");
      else u8g2.print("oF");
      u8g2.setCursor(105, 24);
      u8g2.print("TC0"); //TODO placement of TC
      u8g2.setCursor(105, 34);
      u8g2.print("TC1");
      u8g2.setCursor(99, 44);
      u8g2.print("------");
      u8g2.setCursor(99, 57);
      u8g2.print("------");
      break;
    case 2: //Profile Menu
      u8g2.setCursor(101, 14);
      u8g2.print("PreHt");
      u8g2.setCursor(103, 24);
      u8g2.print("Heat");
      u8g2.setCursor(105, 34);
      u8g2.print("Ref");
      u8g2.setCursor(101, 44);
      u8g2.print("RefKp");
      u8g2.setCursor(103, 57);
      u8g2.print("Cool");
      break;
    case 3: //Tools Menu
      u8g2.setCursor(103, 14);
      u8g2.print("Tune");
      u8g2.setCursor(105, 24);
      u8g2.print("PID");
      u8g2.setCursor(97, 34);
      u8g2.print("TestOut");
      u8g2.setCursor(101, 44);
      u8g2.print("Reset");
      u8g2.setCursor(99, 57);
      u8g2.print("------");
      break;
    case 4: //Start Has been Pressed

      switch (currentState)
      {
        /*case TUNE: {
            u8g2.setCursor(103, 14);
            u8g2.print("Tune");
            break;
          }*/
        case PREHT:
          {
            u8g2.setCursor(101, 14);
            u8g2.print("PreHt");
            break;
          }
        case HEAT:
          {
            u8g2.setCursor(103, 14);
            u8g2.print("Heat");
            break;
          }
        case REF:
          {
            u8g2.setCursor(105, 14);
            u8g2.print("Ref");
            break;
          }
        case REFKP:
          {
            u8g2.setCursor(101, 14);
            u8g2.print("RefKp");
            break;
          }
        case COOL:
          {
            u8g2.setCursor(103, 14);
            u8g2.print("Cool");
            break;
          }
        default:
        case IDLEM:
          {
            u8g2.setCursor(103, 14);
            u8g2.print("Idle");
            break;
          }
      }

      u8g2.setCursor(101, 24);
      if (celsiusMode) u8g2.print((int)shouldBeTemp);
      else u8g2.print((int)cToF(shouldBeTemp));
      u8g2.setCursor(101, 34);
      if (celsiusMode) u8g2.print((rampRateChan0 + rampRateChan1) / 2);
      else u8g2.print(cToF((rampRateChan0 + rampRateChan1) / 2));
      u8g2.setCursor(105, 44);
      u8g2.print(currentReflowSeconds); //Degrees per second
      u8g2.setCursor(103, 57);
      u8g2.print("Stop");
      break;

    case 5: //Edit Values

      switch (profileValueBeingEdited)
      {
        case PREHT:
          {
            u8g2.setCursor(101, 14);
            u8g2.print("PreHt");
            u8g2.setCursor(99, 24);
            u8g2.print("Temp");
            u8g2.setCursor(99, 34);
            if (celsiusMode) u8g2.print(profiles[currentProfile].PreHtTemp);
            else  u8g2.print(cToF(profiles[currentProfile].PreHtTemp));
            u8g2.setCursor(99, 44);
            u8g2.print("Seconds");
            u8g2.setCursor(99, 57);
            u8g2.print(profiles[currentProfile].PreHtTime);
            break;
          }
        case HEAT:
          {
            u8g2.setCursor(103, 14);
            u8g2.print("Heat");
            u8g2.setCursor(99, 24);
            u8g2.print("Temp");
            u8g2.setCursor(99, 34);
            if (celsiusMode) u8g2.print(profiles[currentProfile].HeatTemp);
            else u8g2.print(cToF(profiles[currentProfile].HeatTemp));
            u8g2.print(profiles[currentProfile].HeatTemp);
            u8g2.setCursor(99, 44);
            u8g2.print("Seconds");
            u8g2.setCursor(99, 57);
            u8g2.print(profiles[currentProfile].HeatTime);
            break;
          }
        case REF:
          {
            u8g2.setCursor(105, 14);
            u8g2.print("Ref");
            u8g2.setCursor(99, 24);
            u8g2.print("Temp");
            u8g2.setCursor(99, 34);
            if (celsiusMode) u8g2.print(profiles[currentProfile].RefTemp);
            else u8g2.print(cToF(profiles[currentProfile].RefTemp));
            u8g2.setCursor(99, 44);
            u8g2.print("Seconds");
            u8g2.setCursor(99, 57);
            u8g2.print(profiles[currentProfile].RefTime);
            break;
          }
        case REFKP:
          {
            u8g2.setCursor(101, 14);
            u8g2.print("RefKp");
            u8g2.setCursor(99, 24);
            u8g2.print("Temp");
            u8g2.setCursor(99, 34);
            if (celsiusMode) u8g2.print(profiles[currentProfile].RefKpTemp);
            else u8g2.print(cToF(profiles[currentProfile].RefKpTemp));
            u8g2.setCursor(99, 44);
            u8g2.print("Seconds");
            u8g2.setCursor(99, 57);
            u8g2.print(profiles[currentProfile].RefKpTime);
            break;
          }
        case COOL:
          {
            u8g2.setCursor(103, 14);
            u8g2.print("Cool");
            u8g2.setCursor(99, 24);
            u8g2.print("Temp");
            u8g2.setCursor(99, 34);
            if (celsiusMode) u8g2.print(profiles[currentProfile].CoolTemp);
            else u8g2.print(cToF(profiles[currentProfile].CoolTemp));
            u8g2.setCursor(99, 44);
            u8g2.print("Seconds");
            u8g2.setCursor(99, 57);
            u8g2.print(profiles[currentProfile].CoolTime);
            break;
          }
      }
      break;
    case 6: //Test
      u8g2.setCursor(101, 14);
      u8g2.print("Front");
      u8g2.setCursor(99, 24);
      u8g2.print("Back");
      u8g2.setCursor(99, 34);
      u8g2.print("Stir");
      u8g2.setCursor(99, 44);
      u8g2.print("Vent");
      u8g2.setCursor(99, 57);
      u8g2.print("------");
      break;

    case 7: //PID
      if (currentPIDEditSelection == 1) {
        u8g2.setCursor(101, 14);
        u8g2.print("C1_PH");
        u8g2.setCursor(99, 24);
        u8g2.print("P ");
        u8g2.print(heaterPIDChan1.Kp_PREHEAT);
        u8g2.setCursor(99, 34);
        u8g2.print("I ");
        u8g2.print(heaterPIDChan1.Ki_PREHEAT);
        u8g2.setCursor(99, 44);
        u8g2.print("D ");
        u8g2.print(heaterPIDChan1.Kd_PREHEAT);
        u8g2.setCursor(99, 57);
        u8g2.print("------");
      } else if (currentPIDEditSelection == 0) {
        u8g2.setCursor(101, 14);
        u8g2.print("C0_PH");
        u8g2.setCursor(99, 24);
        u8g2.print("P ");
        u8g2.print(heaterPIDChan0.Kp_PREHEAT);
        u8g2.setCursor(99, 34);
        u8g2.print("I ");
        u8g2.print(heaterPIDChan0.Ki_PREHEAT);
        u8g2.setCursor(99, 44);
        u8g2.print("D ");
        u8g2.print(heaterPIDChan0.Kd_PREHEAT);
        u8g2.setCursor(99, 57);
        u8g2.print("------");
      } else if (currentPIDEditSelection == 3) {
        u8g2.setCursor(101, 14);
        u8g2.print("C1_HT");
        u8g2.setCursor(99, 24);
        u8g2.print("P ");
        u8g2.print(heaterPIDChan1.Kp_SOAK);
        u8g2.setCursor(99, 34);
        u8g2.print("I ");
        u8g2.print(heaterPIDChan1.Ki_SOAK);
        u8g2.setCursor(99, 44);
        u8g2.print("D ");
        u8g2.print(heaterPIDChan1.Kd_SOAK);
        u8g2.setCursor(99, 57);
        u8g2.print("------");
      } else if (currentPIDEditSelection == 2) {
        u8g2.setCursor(101, 14);
        u8g2.print("C0_HT");
        u8g2.setCursor(99, 24);
        u8g2.print("P ");
        u8g2.print(heaterPIDChan0.Kp_SOAK);
        u8g2.setCursor(99, 34);
        u8g2.print("I ");
        u8g2.print(heaterPIDChan0.Ki_SOAK);
        u8g2.setCursor(99, 44);
        u8g2.print("D ");
        u8g2.print(heaterPIDChan0.Kd_SOAK);
        u8g2.setCursor(99, 57);
        u8g2.print("------");
      } else if (currentPIDEditSelection == 5) {
        u8g2.setCursor(101, 14);
        u8g2.print("C1_RF");
        u8g2.setCursor(99, 24);
        u8g2.print("P ");
        u8g2.print(heaterPIDChan1.Kp_REFLOW);
        u8g2.setCursor(99, 34);
        u8g2.print("I ");
        u8g2.print(heaterPIDChan1.Ki_REFLOW);
        u8g2.setCursor(99, 44);
        u8g2.print("D ");
        u8g2.print(heaterPIDChan1.Kd_REFLOW);
        u8g2.setCursor(99, 57);
        u8g2.print("------");
      } else if (currentPIDEditSelection == 4) {
        u8g2.setCursor(101, 14);
        u8g2.print("C0_RF");
        u8g2.setCursor(99, 24);
        u8g2.print("P ");
        u8g2.print(heaterPIDChan0.Kp_REFLOW);
        u8g2.setCursor(99, 34);
        u8g2.print("I ");
        u8g2.print(heaterPIDChan0.Ki_REFLOW);
        u8g2.setCursor(99, 44);
        u8g2.print("D ");
        u8g2.print(heaterPIDChan0.Kd_REFLOW);
        u8g2.setCursor(99, 57);
        u8g2.print("------");
      }
      break;

  }
}

void drawScreen()
{
  // Draw Frame
  u8g2.drawFrame(0, 6, 128, 64 - 6);
  u8g2.drawLine(93, 6, 93, 64);

  // Draw Main Menu Lines

  u8g2.drawLine(93, 16, 128, 16);
  u8g2.drawLine(93, 26, 128, 26);
  u8g2.drawLine(93, 36, 128, 36);
  u8g2.drawLine(93, 46, 128, 46);
}

void drawMenuBox(int index)
{
  u8g2.setDrawColor(2);

  switch (index)
  {
    case 0:
      u8g2.drawBox(95, 8, 31, 7);
      break;
    case 1:
      u8g2.drawBox(95, 18, 31, 7);
      break;
    case 2:
      u8g2.drawBox(95, 28, 31, 7);
      break;
    case 3:
      u8g2.drawBox(95, 38, 31, 7);
      break;
    case 4:
      u8g2.drawBox(95, 48, 31, 14);
      break;
  }

  u8g2.setDrawColor(1); //set to default
}

void refreshMenu()
{
  u8g2.setDrawColor(0);

  u8g2.drawBox(94, 8, 32, 54);

  u8g2.setDrawColor(1); //set to default

  // Draw Main Menu Lines

  u8g2.drawLine(93, 16, 128, 16);
  u8g2.drawLine(93, 26, 128, 26);
  u8g2.drawLine(93, 36, 128, 36);
  u8g2.drawLine(93, 46, 128, 46);
}

void undrawMenuBox(int index)
{
  u8g2.setDrawColor(0);

  switch (index)
  {
    case 0:
      {
        u8g2.drawBox(94, 8, 32, 7);
        break;
      }
    case 1:
      {
        u8g2.drawBox(94, 18, 32, 7);
        break;
      }
    case 2:
      {
        u8g2.drawBox(94, 28, 32, 7);
        break;
      }
    case 3:
      {
        u8g2.drawBox(94, 38, 32, 7);
        break;
      }
    case 4:
      {
        u8g2.drawBox(94, 48, 32, 14);
        break;
      }
  }

  u8g2.setDrawColor(1);
}

void readTemps()
{

  tc0PrevTemp = tc0Temp;
  tc1PrevTemp = tc1Temp;

  int32_t rawData = MAX31855_0.readRawData();
  if (averagesCount < NUMBER_OF_TEMP_AVERAGES) averagesCount++;

  tc0Detect = MAX31855_0.detectThermocouple(rawData);
  if (tc0Detect == MAX31855_THERMOCOUPLE_OK)
  {
    double tc0Value = (double)MAX31855_0.getTemperature(rawData);

    tc0History[tempHistoryHead] = tc0Value;

    tc0Temp = avg(tc0History, averagesCount);
  } else {
    tc0Temp = 0;
  }

  rawData = MAX31855_1.readRawData();
  tc1Detect = MAX31855_1.detectThermocouple(rawData);
  if (tc1Detect == MAX31855_THERMOCOUPLE_OK)
  {
    double tc1Value = (double)MAX31855_1.getTemperature(rawData);

    tc1History[tempHistoryHead] = tc1Value;

    tc1Temp = avg(tc1History, averagesCount);

    boardTemp = MAX31855_1.getColdJunctionTemperature(rawData);
  } else {
    tc1Temp = 0;
  }

  tempHistoryHead++;
  if (tempHistoryHead >= NUMBER_OF_TEMP_AVERAGES)
    tempHistoryHead = 0;
}

void updateTemps()
{
  if (errorTimeout <= 0) {
    if (tc0Detect != MAX31855_THERMOCOUPLE_OK)
    {
      u8g2log.print("TC0:NC    ");
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
      u8g2log.print("TC1:NC    ");
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

unsigned long pollRate = 1000;


void disableReflow()
{
  reflow = 0;
  currentState = IDLEM;
  pollRate = 1000;

}

double previousRateDisplayTemp0 = 0;

double previousRateDisplayTemp1 = 0;

boolean tuning = false;

boolean currentStateAutotuned = false;

void startReflow()
{
  reflowStartTime = millis();
  previousTempDisplayUpdate = 0;
  reflow = 1;
  //if (currentState != TUNE)
  currentState = PREHT;

  shouldBeTemp = (int)((tc0Temp + tc1Temp) / 2);
  rampRateChan0 = 0;
  rampRateChan1 = 0;


  previousRateDisplayTemp0 = tc0Temp;

  previousRateDisplayTemp1 = tc1Temp;

  pollRate = 200;
  PIDChan0.SetSampleTime(200);
  PIDChan1.SetSampleTime(200);
}


void printError()
{


  // Refresh the screen
  u8g2log.print("\r");
  u8g2.drawLog(0, 5, u8g2log);

  u8g2.updateDisplayArea(0, 0, 128, 6);

  errorTimeout = 1;
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
            //buzzer(SPK);
            buttonStateOK = 0;
            buttonStatePLUS = 0;
            buttonStateMINUS = 0;
            profileValueChanged = true;
            switch (profileValueBeingEdited)
            {
              case PREHT:
                {

                  profiles[currentProfile].PreHtTemp = profiles[currentProfile].PreHtTemp - 1;
                  if (profiles[currentProfile].PreHtTemp < 0) profiles[currentProfile].PreHtTemp = 250;

                  break;
                }
              case HEAT:
                {
                  profiles[currentProfile].HeatTemp = profiles[currentProfile].HeatTemp - 1;
                  if (profiles[currentProfile].HeatTemp < 0) profiles[currentProfile].HeatTemp = 250;
                  break;
                }
              case REF:
                {
                  profiles[currentProfile].RefTemp = profiles[currentProfile].RefTemp - 1;
                  if (profiles[currentProfile].RefTemp < 0) profiles[currentProfile].RefTemp = 250;
                  break;
                }
              case REFKP:
                {
                  profiles[currentProfile].RefKpTemp = profiles[currentProfile].RefKpTemp - 1;
                  if (profiles[currentProfile].RefKpTemp < 0) profiles[currentProfile].RefKpTemp = 250;
                  break;
                }
              case COOL:
                {
                  profiles[currentProfile].CoolTemp = profiles[currentProfile].CoolTemp - 1;
                  if (profiles[currentProfile].CoolTemp < 0) profiles[currentProfile].CoolTemp = 250;
                  break;
                }
            }
            refreshMenu();
            drawMenu(currentMenuID);
            drawMenuBox(menuState);
            u8g2.updateDisplay();
          }
        }
        else if (menuState == 4 && digitalRead(OK) == 0)
        {
          delay(1); //noise reduction
          if (digitalRead(OK) == 0)
          {
            //buzzer(SPK);
            buttonStateOK = 0;
            buttonStatePLUS = 0;
            buttonStateMINUS = 0;
            profileValueChanged = true;
            switch (profileValueBeingEdited)
            {
              case PREHT:
                {

                  profiles[currentProfile].PreHtTime = profiles[currentProfile].PreHtTime - 1;
                  if (profiles[currentProfile].PreHtTime < 0) profiles[currentProfile].PreHtTime = 200;

                  break;
                }
              case HEAT:
                {
                  profiles[currentProfile].HeatTime = profiles[currentProfile].HeatTime - 1;
                  if (profiles[currentProfile].HeatTime < 0) profiles[currentProfile].HeatTime = 200;
                  break;
                }
              case REF:
                {
                  profiles[currentProfile].RefTime = profiles[currentProfile].RefTime - 1;
                  if (profiles[currentProfile].RefTime < 0) profiles[currentProfile].RefTime = 200;
                  break;
                }
              case REFKP:
                {
                  profiles[currentProfile].RefKpTime = profiles[currentProfile].RefKpTime - 1;
                  if (profiles[currentProfile].RefKpTime < 0) profiles[currentProfile].RefKpTime = 200;
                  break;
                }
              case COOL:
                {
                  profiles[currentProfile].CoolTime = profiles[currentProfile].CoolTime - 1;
                  if (profiles[currentProfile].CoolTime < 0) profiles[currentProfile].CoolTime = 200;
                  break;
                }
            }
            refreshMenu();
            drawMenu(currentMenuID);
            drawMenuBox(menuState);
            u8g2.updateDisplay();
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
            //buzzer(SPK);
            buttonStateOK = 0;
            buttonStatePLUS = 0;
            buttonStateMINUS = 0;
            profileValueChanged = true;
            switch (profileValueBeingEdited)
            {
              case PREHT:
                {

                  profiles[currentProfile].PreHtTemp = profiles[currentProfile].PreHtTemp + 1;
                  if (profiles[currentProfile].PreHtTemp > 250) profiles[currentProfile].PreHtTemp = 0;

                  break;
                }
              case HEAT:
                {
                  profiles[currentProfile].HeatTemp = profiles[currentProfile].HeatTemp + 1;
                  if (profiles[currentProfile].HeatTemp > 250) profiles[currentProfile].HeatTemp = 0;
                  break;
                }
              case REF:
                {
                  profiles[currentProfile].RefTemp = profiles[currentProfile].RefTemp + 1;
                  if (profiles[currentProfile].RefTemp > 250) profiles[currentProfile].RefTemp = 0;
                  break;
                }
              case REFKP:
                {
                  profiles[currentProfile].RefKpTemp = profiles[currentProfile].RefKpTemp + 1;
                  if (profiles[currentProfile].RefKpTemp > 250) profiles[currentProfile].RefKpTemp = 0;
                  break;
                }
              case COOL:
                {
                  profiles[currentProfile].CoolTemp = profiles[currentProfile].CoolTemp + 1;
                  if (profiles[currentProfile].CoolTemp > 250) profiles[currentProfile].CoolTemp = 0;
                  break;
                }
            }
            refreshMenu();
            drawMenu(currentMenuID);
            drawMenuBox(menuState);
            u8g2.updateDisplay();
          }
        }
        else if (menuState == 4 && digitalRead(OK) == 0)
        {
          delay(1); //noise reduction
          if (digitalRead(OK) == 0)
          {
            //buzzer(SPK);
            buttonStateOK = 0;
            buttonStatePLUS = 0;
            buttonStateMINUS = 0;
            profileValueChanged = true;
            switch (profileValueBeingEdited)
            {
              case PREHT:
                {

                  profiles[currentProfile].PreHtTime = profiles[currentProfile].PreHtTime + 1;
                  if (profiles[currentProfile].PreHtTime > 200) profiles[currentProfile].PreHtTime = 0;

                  break;
                }
              case HEAT:
                {
                  profiles[currentProfile].HeatTime = profiles[currentProfile].HeatTime + 1;
                  if (profiles[currentProfile].HeatTime > 200) profiles[currentProfile].HeatTime = 0;
                  break;
                }
              case REF:
                {
                  profiles[currentProfile].RefTime = profiles[currentProfile].RefTime + 1;
                  if (profiles[currentProfile].RefTime > 200) profiles[currentProfile].RefTime = 0;
                  break;
                }
              case REFKP:
                {
                  profiles[currentProfile].RefKpTime = profiles[currentProfile].RefKpTime + 1;
                  if (profiles[currentProfile].RefKpTime > 200) profiles[currentProfile].RefKpTime = 0;
                  break;
                }
              case COOL:
                {
                  profiles[currentProfile].CoolTime = profiles[currentProfile].CoolTime + 1;
                  if (profiles[currentProfile].CoolTime > 200) profiles[currentProfile].CoolTime = 0;
                  break;
                }
            }

            refreshMenu();
            drawMenu(currentMenuID);
            drawMenuBox(menuState);
            u8g2.updateDisplay();
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

    buzzer(SPK);
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
            drawProfile();
            storedVar.profile = currentProfile;
            if (previousSavedProfile != currentProfile)
            {
              writeTimeout = 10000;
              saved = false;
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
            if (tc1Temp < START_TEMP_MAX && tc0Temp < START_TEMP_MAX)
            {
              //TODO check for thermocouple
              drawProfile();
              u8g2.updateDisplay();

              currentMenuID = 4;

              //Update menu each second
              enableMenu = 0;
              Serial.println("");
              Serial.println ("Reflow Started");
              Serial.print ("Profile Settings (");
              Serial.print(currentProfile);
              Serial.println(")");

              Serial.print("PreHt: ");
              if (celsiusMode) {
                Serial.print(profiles[currentProfile].PreHtTemp);
                Serial.print("°C, ");
              } else {
                Serial.print(cToF(profiles[currentProfile].PreHtTemp));
                Serial.print("°F, ");
              }
              Serial.print(profiles[currentProfile].PreHtTime);
              Serial.println("s");

              Serial.print("Heat: ");
              if (celsiusMode) {
                Serial.print(profiles[currentProfile].HeatTemp);
                Serial.print("°C, ");
              } else {
                Serial.print(cToF(profiles[currentProfile].HeatTemp));
                Serial.print("°F, ");
              }
              Serial.print(profiles[currentProfile].HeatTime);
              Serial.println("s");

              Serial.print("Ref: ");
              if (celsiusMode) {
                Serial.print(profiles[currentProfile].RefTemp);
                Serial.print("°C, ");
              } else {
                Serial.print(cToF(profiles[currentProfile].RefTemp));
                Serial.print("°F, ");
              }
              Serial.print(profiles[currentProfile].RefTime);
              Serial.println("s");

              Serial.print("RefKp: ");
              if (celsiusMode) {
                Serial.print(profiles[currentProfile].RefKpTemp);
                Serial.print("°C, ");
              } else {
                Serial.print(cToF(profiles[currentProfile].RefKpTemp));
                Serial.print("°F, ");
              }
              Serial.print(profiles[currentProfile].RefKpTime);
              Serial.println("s");

              Serial.print("Cool: ");
              if (celsiusMode) {
                Serial.print(profiles[currentProfile].CoolTemp);
                Serial.print("°C, ");
              } else {
                Serial.print(cToF(profiles[currentProfile].CoolTemp));
                Serial.print("°F, ");
              }
              Serial.print(profiles[currentProfile].CoolTime);
              Serial.println("s");

              Serial.println("");

              Serial.println("Channel 0 PID Preheat");

              Serial.print("P: ");
              Serial.println(heaterPIDChan0.Kp_PREHEAT);
              Serial.print("I: ");
              Serial.println(heaterPIDChan0.Ki_PREHEAT);
              Serial.print("D: ");
              Serial.println(heaterPIDChan0.Kd_PREHEAT);

              Serial.println("");

              Serial.println("Channel 1 PID Preheat");

              Serial.print("P: ");
              Serial.println(heaterPIDChan1.Kp_PREHEAT);
              Serial.print("I: ");
              Serial.println(heaterPIDChan1.Ki_PREHEAT);
              Serial.print("D: ");
              Serial.println(heaterPIDChan1.Kd_PREHEAT);

              Serial.println("");

              Serial.println("Channel 0 PID Heat");

              Serial.print("P: ");
              Serial.println(heaterPIDChan0.Kp_SOAK);
              Serial.print("I: ");
              Serial.println(heaterPIDChan0.Ki_SOAK);
              Serial.print("D: ");
              Serial.println(heaterPIDChan0.Kd_SOAK);

              Serial.println("");

              Serial.println("Channel 1 PID Heat");

              Serial.print("P: ");
              Serial.println(heaterPIDChan1.Kp_SOAK);
              Serial.print("I: ");
              Serial.println(heaterPIDChan1.Ki_SOAK);
              Serial.print("D: ");
              Serial.println(heaterPIDChan1.Kd_SOAK);

              Serial.println("");

              Serial.println("Channel 0 PID Reflow");

              Serial.print("P: ");
              Serial.println(heaterPIDChan0.Kp_REFLOW);
              Serial.print("I: ");
              Serial.println(heaterPIDChan0.Ki_REFLOW);
              Serial.print("D: ");
              Serial.println(heaterPIDChan0.Kd_REFLOW);

              Serial.println("");

              Serial.println("Channel 1 PID Reflow");

              Serial.print("P: ");
              Serial.println(heaterPIDChan1.Kp_REFLOW);
              Serial.print("I: ");
              Serial.println(heaterPIDChan1.Ki_REFLOW);
              Serial.print("D: ");
              Serial.println(heaterPIDChan1.Kd_REFLOW);


              Serial.println("");
              Serial.println("Mode,Seconds,SetPoint,TC0,RateTC0,PWMOut0,TC1,RateTC1,PWMOut1");
              Serial.println("");

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
        //PIDTuneChan0.Cancel();
        //PIDTuneChan1.Cancel();
        tuning = false;
      }

      disableReflow();

      printError();
    } else if (currentMenuID == 2) //Editing profile value
    {
      currentMenuID = 5;

      //Value being edited
      profileValueBeingEdited = menuState;

      if (menuState != 2 && menuState != 4)
      {
        prevMenuState = menuState;
        menuState = 2;
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
          if (tc1Temp < START_TEMP_MAX && tc0Temp < START_TEMP_MAX)
          {
            if (tuning != true) { //Set the output to the desired starting frequency.
              Serial.println("");
              Serial.println("Tuning Started");
              Serial.println("");
              Serial.println("Mode,Seconds,SetPoint,TC0,RateTC0,PWMOut0,TC1,RateTC1,PWMOut1");
              Serial.println("");
              tuning = true;
              currentState = PREHT;

              /*OutputChan0 = aTuneStartValueChan0;
                PIDTuneChan0.SetNoiseBand(aTuneNoiseChan0);
                PIDTuneChan0.SetOutputStep(aTuneStepChan0);
                PIDTuneChan0.SetLookbackSec((int)aTuneLookBackChan0);

                OutputChan1 = aTuneStartValueChan1;
                PIDTuneChan1.SetNoiseBand(aTuneNoiseChan1);
                PIDTuneChan1.SetOutputStep(aTuneStepChan1);
                PIDTuneChan1.SetLookbackSec((int)aTuneLookBackChan1);*/

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
            restoreDefaults();
            loadSettings();
            saved = false;
            writeTimeout = 1000;
            //u8g2log.print("\fSettings Restored");
            //printError();
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
            //digitalWrite(FRONT_HEATER, 0);
          } else {
            frontState = 0;
            //digitalWrite(FRONT_HEATER, 1);
          }
          break;
        case 1: //Back
          if (backState == 0) //Turn on
          {
            backState = 1;
            //digitalWrite(BACK_HEATER, 0);
          } else {
            backState = 0;
            //digitalWrite(BACK_HEATER, 1);
          }
          break;
        case 2: //Stir
          if (convectionState == 0) //Turn on
          {
            convectionState = 1;
            //digitalWrite(CONVECTION, 0);
          } else {
            convectionState = 0;
            //digitalWrite(CONVECTION, 1);
          }
          break;
        case 3: //Vent
          if (exhaustState == 0) //Turn on
          {
            exhaustState = 1;
            //digitalWrite(EXHAUST, 0);
          } else {
            exhaustState = 0;
            //digitalWrite(EXHAUST, 1);
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
          storedVar.celsiusMode = celsiusMode;
          saved = false;
          writeTimeout = 1000;
          break;
      }
    }

    refreshMenu();
    drawMenu(currentMenuID);
    drawMenuBox(menuState);
    u8g2.updateDisplay();
  }

  if (buttonStateBACK == buttonStateTimeout)
  {
    buzzer(SPK);

    switch (currentMenuID)
    {
      case 1: //Config
        currentMenuID = 0;
        break;
      case 2: //Profile
        currentMenuID = 0;
        if (profileValueChanged)
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

        profileValueChanged = false;
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
          //PIDTuneChan0.Cancel();
          //PIDTuneChan1.Cancel();
          tuning = false;
        }

        disableReflow();

        printError();
        break;
      case 5: //Editing a value

        currentMenuID = 2;

        drawProfile();
        u8g2.updateDisplay();

        break;
      case 6: //Testing outputs
        currentMenuID = 3;
        /* Disable outputs */
        /*digitalWrite(FRONT_HEATER, 1);
          digitalWrite(BACK_HEATER, 1);
          digitalWrite(CONVECTION, 1);
          digitalWrite(EXHAUST, 1);*/
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
      drawCurrentTemp((tc1Temp + tc0Temp) / 2, currentReflowSeconds);
    }
  }

  if (currentTime - previousTimeReflowDisplayUpdate >= 1000 )
  {

    previousTimeReflowDisplayUpdate = currentTime;

    if (reflow) {

      refreshMenu();
      drawMenu(currentMenuID);

      u8g2.updateDisplay();

      switch (currentState)
      {
        /*case TUNE: {
            Serial.print("Tune,");
            break;
          }*/
        case PREHT:
          {
            Serial.print("PreHt,");
            break;
          }
        case HEAT:
          {
            Serial.print("Heat,");
            break;
          }
        case REF:
          {
            Serial.print("Ref,");
            break;
          }
        case REFKP:
          {
            Serial.print("RefKp,");
            break;
          }
        case COOL:
          {
            Serial.print("Cool,");
            break;
          }
      }

      Serial.print(currentReflowSeconds);
      Serial.print(",");
      if (celsiusMode) Serial.print(shouldBeTemp);
      else Serial.print(cToF(shouldBeTemp));
      Serial.print(",");
      if (celsiusMode) Serial.print(tc0Temp);
      else Serial.print(cToF(tc0Temp));
      Serial.print(",");
      if (celsiusMode) Serial.print(rampRateChan0);
      else Serial.print(cToF(rampRateChan0));
      Serial.print(",");
      Serial.print(OutputChan0);
      Serial.print(",");
      if (celsiusMode) Serial.print(tc1Temp);
      else Serial.print(cToF(tc1Temp));
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

    rampRateChan0 = (tc0Temp - previousRateDisplayTemp0) * 1000 / (currentTempLoopMillis - prevTempLoopMillis);
    rampRateChan1 = (tc1Temp - previousRateDisplayTemp1) * 1000 / (currentTempLoopMillis - prevTempLoopMillis);

    previousRateDisplayTemp0 = tc0Temp;

    previousRateDisplayTemp1 = tc1Temp;

    InputChan0 = tc0Temp; // update the variable the PID reads

    InputChan1 = tc1Temp; // update the variable the PID reads

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
            SetpointChan0 = tc0Temp;

            //if (!tuning) OutputChan1 = 80;
            OutputChan1 = 80;
            PIDChan1.SetMode(AUTOMATIC);
            PIDChan1.SetControllerDirection(DIRECT);
            PIDChan1.SetTunings(heaterPIDChan1.Kp_PREHEAT, heaterPIDChan1.Ki_PREHEAT, heaterPIDChan1.Kd_PREHEAT);
            SetpointChan1 = tc1Temp;

            startTemp = (tc0Temp + tc1Temp) / 2;
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
            //PIDTuneChan0.setpoint = shouldBeTemp;
            //PIDTuneChan1.setpoint = shouldBeTemp;
            //int8_t val0 = PIDTuneChan0.Runtime();
            //int8_t val1 = PIDTuneChan1.Runtime();

            tunerChan0.setTargetInputValue(shouldBeTemp);
            tunerChan1.setTargetInputValue(shouldBeTemp);

            OutputChan0 = tunerChan0.tunePID(InputChan0);
            OutputChan1 = tunerChan1.tunePID(InputChan1);

            /*if (currentStateAutotuned)
              {
              OutputChan0 = 100;
              OutputChan1 = 100;
              }*/

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
              Serial.println("Channel 0 PID Preheat");

              Serial.print("P: ");
              Serial.println(heaterPIDChan0.Kp_PREHEAT);
              Serial.print("I: ");
              Serial.println(heaterPIDChan0.Ki_PREHEAT);
              Serial.print("D: ");
              Serial.println(heaterPIDChan0.Kd_PREHEAT);

              Serial.println("");

              Serial.println("Channel 1 PID Preheat");

              Serial.print("P: ");
              Serial.println(heaterPIDChan1.Kp_PREHEAT);
              Serial.print("I: ");
              Serial.println(heaterPIDChan1.Ki_PREHEAT);
              Serial.print("D: ");
              Serial.println(heaterPIDChan1.Kd_PREHEAT);

              printError();

            }

            /* //Get to next temp range before tuning further
              if (tc0Temp > profiles[currentProfile].PreHtTemp - 1 && tc1Temp > profiles[currentProfile].PreHtTemp - 1 && currentStateAutotuned) {
               currentState = HEAT;
              }*/

          } else {

            if (tc0Temp > profiles[currentProfile].PreHtTemp - 1 && tc1Temp > profiles[currentProfile].PreHtTemp - 1) {
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

            SetpointChan0 = tc0Temp;

            SetpointChan1 = tc1Temp;

            PIDChan0.SetTunings(heaterPIDChan0.Kp_SOAK, heaterPIDChan0.Ki_SOAK, heaterPIDChan0.Kd_SOAK);
            PIDChan1.SetTunings(heaterPIDChan1.Kp_SOAK, heaterPIDChan1.Ki_SOAK, heaterPIDChan1.Kd_SOAK);

            if (tuning)
            {
              tunerChan0.setTargetInputValue(profiles[currentProfile].HeatTemp);
              tunerChan1.setTargetInputValue(profiles[currentProfile].HeatTemp);

              tunerChan0.startTuningLoop();
              tunerChan1.startTuningLoop();
            }

            startTemp = (tc0Temp + tc1Temp) / 2;
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

            /*if (currentStateAutotuned)
            {
              OutputChan0 = 100;
              OutputChan1 = 100;
            }*/

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
              Serial.println("Channel 0 PID Heat");

              Serial.print("P: ");
              Serial.println(heaterPIDChan0.Kp_SOAK);
              Serial.print("I: ");
              Serial.println(heaterPIDChan0.Ki_SOAK);
              Serial.print("D: ");
              Serial.println(heaterPIDChan0.Kd_SOAK);

              Serial.println("");

              Serial.println("Channel 1 PID Heat");

              Serial.print("P: ");
              Serial.println(heaterPIDChan1.Kp_SOAK);
              Serial.print("I: ");
              Serial.println(heaterPIDChan1.Ki_SOAK);
              Serial.print("D: ");
              Serial.println(heaterPIDChan1.Kd_SOAK);

              printError();

            }

            /*//Get to next temp range before tuning further
              if (tc0Temp >= profiles[currentProfile].HeatTemp - 1 && tc1Temp >= profiles[currentProfile].HeatTemp - 1 && currentStateAutotuned) {
              currentState = REF;
              }*/

          } else {

            if (tc0Temp >= profiles[currentProfile].HeatTemp - 1 && tc1Temp >= profiles[currentProfile].HeatTemp - 1) {
              currentState = REF;
            }
          }
          break;
        }

      case REF:
        {
          if (currentState != previousState) { //State changed
            convectionState = 1;

            SetpointChan0 = tc0Temp;

            SetpointChan1 = tc1Temp;

            PIDChan0.SetTunings(heaterPIDChan0.Kp_REFLOW, heaterPIDChan0.Ki_REFLOW, heaterPIDChan0.Kd_REFLOW);
            PIDChan1.SetTunings(heaterPIDChan1.Kp_REFLOW, heaterPIDChan1.Ki_REFLOW, heaterPIDChan1.Kd_REFLOW);

            if (tuning)
            {
              tunerChan0.setTargetInputValue(profiles[currentProfile].RefTemp);
              tunerChan1.setTargetInputValue(profiles[currentProfile].RefTemp);

              tunerChan0.startTuningLoop();
              tunerChan1.startTuningLoop();
            }

            startTemp = (tc0Temp + tc1Temp) / 2;
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

            /*if (currentStateAutotuned)
            {
              OutputChan0 = 100;
              OutputChan1 = 100;
            }*/

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
              Serial.println("Channel 0 PID Reflow");

              Serial.print("P: ");
              Serial.println(heaterPIDChan0.Kp_REFLOW);
              Serial.print("I: ");
              Serial.println(heaterPIDChan0.Ki_REFLOW);
              Serial.print("D: ");
              Serial.println(heaterPIDChan0.Kd_REFLOW);

              Serial.println("");

              Serial.println("Channel 1 PID Reflow");

              Serial.print("P: ");
              Serial.println(heaterPIDChan1.Kp_REFLOW);
              Serial.print("I: ");
              Serial.println(heaterPIDChan1.Ki_REFLOW);
              Serial.print("D: ");
              Serial.println(heaterPIDChan1.Kd_REFLOW);

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

            if (tc0Temp >= profiles[currentProfile].RefTemp - 2 && tc1Temp >= profiles[currentProfile].RefTemp - 2 ) {
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

            /*if (tuning)
              {
              tunerChan0.setTargetInputValue(profiles[currentProfile].RefKpTemp);
              tunerChan1.setTargetInputValue(profiles[currentProfile].RefKpTemp);

              tunerChan0.startTuningLoop();
              tunerChan1.startTuningLoop();
              }*/

            startTemp = (tc0Temp + tc1Temp) / 2;
            startStateMillis = currentTime;

          }
          previousState = currentState;

          /*if (tuning) {
            tunerChan0.setTargetInputValue(profiles[currentProfile].RefKpTemp);
            tunerChan1.setTargetInputValue(profiles[currentProfile].RefKpTemp);

            OutputChan0 = tunerChan0.tunePID(InputChan0);
            OutputChan1 = tunerChan1.tunePID(InputChan1);

            if (tunerChan0.isFinished() && tunerChan1.isFinished())
            {

              Serial.println("Tuning Complete");
              u8g2log.print("\fTuning Complete");
              writeTimeout = 1000;
              saved = false;
              currentState = IDLEM; //Go straight to idle as we aren't going to tune refkp or cool

            }*/
          // } else
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
            SetpointChan0 = tc0Temp;

            PIDChan1.SetControllerDirection(REVERSE);
            PIDChan1.SetTunings(fanPID.Kp, fanPID.Ki, fanPID.Kd);
            SetpointChan1 = tc1Temp;

            //Start fan up full so it doesn't have to catch up with PID on initial state change
            OutputChan0 = 100;
            OutputChan1 = 100;

            startTemp = (tc0Temp + tc1Temp) / 2;

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

          if (tc0Temp <= profiles[currentProfile].CoolTemp + 2 || tc1Temp <= profiles[currentProfile].CoolTemp + 2 ) {
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

          if (tc1Temp > START_TEMP_MAX || tc0Temp > START_TEMP_MAX)
          {
            OutputChan0 = 100;
            OutputChan1 = 100;
            /*if (tc1Temp > 80 || tc0Temp > 80) //If greater than 80 degrees
              {
              OutputChan0 = 100;
              OutputChan1 = 100;
              } else {
              OutputChan0 = 50;
              OutputChan1 = 50;
              }*/
          } else if (tc1Temp < START_TEMP_MAX - 10 && tc0Temp < START_TEMP_MAX - 10) { //10 degree hysteresis
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

  checkBootloader();
  delay(1);
}
