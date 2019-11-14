#include "helper.h"
#include "variables.h"

#ifndef _MENU_H
#define _MENU_H

U8G2_KS0108_128X64_F u8g2(U8G2_R0, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, /*enable=*/E, /*dc=*/DI, /*cs0=*/CS1, /*cs1=*/CS2, /*cs2=*/U8X8_PIN_NONE, /* reset=*/RST);
int errorTimeout = 0;
// assume 4x6 font, define width and height
#define U8LOG_WIDTH 32
#define U8LOG_HEIGHT 1
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];
U8G2LOG u8g2log;

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

void setCursor(int numChars, int lineNumber)
{
  int y = 14;
  switch (lineNumber)
  {
    case 0: y = 14; break;
    case 1: y = 24; break;
    case 2: y = 34; break;
    case 3: y = 44; break;
    case 4: y = 57; break;
  }

  //Pixels 94 - 128
  int x = 0;
  switch (numChars)
  {
    case 1: x = 109; break;
    case 2: x = 107; break;
    case 3: x = 105; break;
    case 4: x = 103; break;
    case 5: x = 101; break;
    case 6: x = 99; break;
    case 7: x = 97; break;
    case 8: x = 95; break;
  }

  u8g2.setCursor(x, y);
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

//This translates from normal cartesian lines to LCD library x,y pixels inside profile draw area
void profileBoxPixel(int x, int y)
{
  //start y 7, size 56
  //start x 1, size 92
  //u8g2.drawFrame(1, 7, 92, 56);

  u8g2.drawPixel(x + 1, 56 - y + 7);
}

//This translates from normal cartesian lines to LCD library x,y lines inside profile draw area
void profileBoxLine(int x1, int y1, int x2, int y2)
{
  //start y 7, size 56
  //start x 1, size 92
  //u8g2.drawFrame(1, 7, 92, 56);

  u8g2.drawLine(x1 + 1, 56 - y1 + 7, x2 + 1, 56 - y2 + 7);
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

void printPID(double P, double I, double D)
{
  setCursor(7, 1);
  u8g2.print("P");
  u8g2.print(P);
  setCursor(7, 2);
  u8g2.print("I");
  u8g2.print(I);
  setCursor(7, 3);
  u8g2.print("D");
  u8g2.print(D);
}

void printTestMenu()
{
  setCursor(5, 0);
  u8g2.print("Front");
  setCursor(4, 1);
  u8g2.print("Back");
  setCursor(4, 2);
  u8g2.print("Stir");
  setCursor(4, 3);
  u8g2.print("Vent");
  setCursor(6, 4);
  u8g2.print("------");
}

void printConfigMenu(bool celsiusMode, int tcState, bool beepState) {
  setCursor(2, 0);
  if (celsiusMode) u8g2.print("oC");
  else u8g2.print("oF");
  setCursor(7, 1);
  if (tcState == 0)
    u8g2.print("B:0,F:1");
  else if (tcState == 1)
    u8g2.print("B:1,F:0");
  else if (tcState == 2)
    u8g2.print("B:A,F:A");
  if (beepState == true)
  { 
    setCursor(4, 2);
    u8g2.print("Beep");
  } else {
    setCursor(6, 2);
    u8g2.print("NoBeep");
  }
  setCursor(6, 3);
  u8g2.print("------");
  setCursor(6, 4);
  u8g2.print("------");
}

void printCurrentState(int currentState)
{
  switch (currentState)
  {
    case PREHT:
      {
        setCursor(5, 0);
        u8g2.print("PreHt");
        break;
      }
    case HEAT:
      {
        setCursor(4, 0);
        u8g2.print("Heat");
        break;
      }
    case REF:
      {
        setCursor(3, 0);
        u8g2.print("Ref");
        break;
      }
    case REFKP:
      {
        setCursor(5, 0);
        u8g2.print("RefKp");
        break;
      }
    case COOL:
      {
        setCursor(4, 0);
        u8g2.print("Cool");
        break;
      }
    default:
    case IDLEM:
      {
        setCursor(4, 0);
        u8g2.print("Idle");
        break;
      }
  }
}

void printEditValues(int profileValueBeingEdited, ReflowProfile * profiles, int currentProfile, bool celsiusMode) {
  switch (profileValueBeingEdited)
  {
    case PREHT:
      {
        setCursor(5, 0);
        u8g2.print("PreHt");
        setCursor(4, 1);
        u8g2.print("Temp");

        if (celsiusMode)
        {
          setCursor(intLen(profiles[currentProfile].PreHtTemp), 2);
          u8g2.print(profiles[currentProfile].PreHtTemp);
        }
        else  {
          setCursor(intLen(profiles[currentProfile].PreHtTemp), 2);
          u8g2.print(cToF(profiles[currentProfile].PreHtTemp));
        }
        setCursor(7, 3);
        u8g2.print("Seconds");
        setCursor(intLen(profiles[currentProfile].PreHtTime), 4);
        u8g2.print(profiles[currentProfile].PreHtTime);
        break;
      }
    case HEAT:
      {
        setCursor(4, 0);
        u8g2.print("Heat");
        setCursor(4, 1);
        u8g2.print("Temp");

        if (celsiusMode) {
          setCursor(intLen(profiles[currentProfile].HeatTemp), 2);
          u8g2.print(profiles[currentProfile].HeatTemp);
        }
        else {
          setCursor(intLen(cToF(profiles[currentProfile].HeatTemp)), 2);
          u8g2.print(cToF(profiles[currentProfile].HeatTemp));
        }
        setCursor(7, 3);
        u8g2.print("Seconds");
        setCursor(intLen(profiles[currentProfile].HeatTime), 4);
        u8g2.print(profiles[currentProfile].HeatTime);
        break;
      }
    case REF:
      {
        setCursor(3, 0);
        u8g2.print("Ref");
        setCursor(4, 1);
        u8g2.print("Temp");
        if (celsiusMode) {
          setCursor(intLen(profiles[currentProfile].RefTemp), 2);
          u8g2.print(profiles[currentProfile].RefTemp);
        }
        else {
          setCursor(intLen(cToF(profiles[currentProfile].RefTemp)), 2);
          u8g2.print(cToF(profiles[currentProfile].RefTemp));
        }
        setCursor(7, 3);
        u8g2.print("Seconds");
        setCursor(intLen(profiles[currentProfile].RefTime), 4);
        u8g2.print(profiles[currentProfile].RefTime);
        break;
      }
    case REFKP:
      {
        setCursor(5, 0);
        u8g2.print("RefKp");
        setCursor(4, 1);
        u8g2.print("Temp");
        if (celsiusMode) {
          setCursor(intLen(profiles[currentProfile].RefKpTemp), 2);
          u8g2.print(profiles[currentProfile].RefKpTemp);
        }
        else {
          setCursor(intLen(cToF(profiles[currentProfile].RefKpTemp)), 2);
          u8g2.print(cToF(profiles[currentProfile].RefKpTemp));
        }
        setCursor(7, 3);
        u8g2.print("Seconds");
        setCursor(intLen(profiles[currentProfile].RefKpTime), 4);
        u8g2.print(profiles[currentProfile].RefKpTime);
        break;
      }
    case COOL:
      {
        setCursor(4, 0);
        u8g2.print("Cool");
        setCursor(4, 1);
        u8g2.print("Temp");
        if (celsiusMode) {
          setCursor(intLen(profiles[currentProfile].CoolTemp), 2);
          u8g2.print(profiles[currentProfile].CoolTemp);
        }
        else {
          setCursor(intLen(cToF(profiles[currentProfile].CoolTemp)), 2);
          u8g2.print(cToF(profiles[currentProfile].CoolTemp));
        }
        setCursor(7, 3);
        u8g2.print("Seconds");
        setCursor(intLen(profiles[currentProfile].CoolTime), 4);
        u8g2.print(profiles[currentProfile].CoolTime);
        break;
      }
  }
}

void printMainMenu(int currentProfile)
{
  setCursor(6, 0);
  u8g2.print("Config");
  setCursor(6, 1);
  u8g2.print("Prof ");
  u8g2.print(currentProfile);
  setCursor(7, 2);
  u8g2.print("Profile");
  setCursor(5, 3);
  u8g2.print("Tools");
  setCursor(5, 4);
  u8g2.print("Start");
}

void printToolsMenu()
{
  setCursor(4, 0);
  u8g2.print("Tune");
  setCursor(3, 1);
  u8g2.print("PID");
  setCursor(7, 2);
  u8g2.print("TestOut");
  setCursor(5, 3);
  u8g2.print("Reset");
  setCursor(6, 4);
  u8g2.print("------");
}

void printProfileMenu()
{
  setCursor(5, 0);
  u8g2.print("PreHt");
  setCursor(4, 1);
  u8g2.print("Heat");
  setCursor(3, 2);
  u8g2.print("Ref");
  setCursor(5, 3);
  u8g2.print("RefKp");
  setCursor(4, 4);
  u8g2.print("Cool");
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

void drawProfile(ReflowProfile * profiles, double tcTemp, int currentProfile)
{
  clearProfile();

  if (tcTemp > 0)
  {
    profileLine(tcTemp, profiles[currentProfile].PreHtTemp, 0, profiles[currentProfile].PreHtTime);
  } else {
    profileLine(0, profiles[currentProfile].PreHtTemp, 0, profiles[currentProfile].PreHtTime);
  }
  unsigned long tempStart = 0;
  profileLine(tcTemp, profiles[currentProfile].PreHtTemp, tempStart, tempStart + profiles[currentProfile].PreHtTime);
  tempStart += profiles[currentProfile].PreHtTime;
  profileLine(profiles[currentProfile].PreHtTemp, profiles[currentProfile].HeatTemp, tempStart, tempStart + profiles[currentProfile].HeatTime);
  tempStart += profiles[currentProfile].HeatTime;
  profileLine(profiles[currentProfile].HeatTemp, profiles[currentProfile].RefTemp, tempStart, tempStart + profiles[currentProfile].RefTime);
  tempStart += profiles[currentProfile].RefTime;
  profileLine(profiles[currentProfile].RefTemp, profiles[currentProfile].RefKpTemp, tempStart, tempStart + profiles[currentProfile].RefKpTime);
  tempStart += profiles[currentProfile].RefKpTime;
  profileLine(profiles[currentProfile].RefKpTemp, profiles[currentProfile].CoolTemp, tempStart, tempStart + profiles[currentProfile].CoolTime);
}

void printCurrentPIDMenu(PID3_t heaterPIDChan0, PID3_t heaterPIDChan1, int currentPIDEditSelection) {
  setCursor(5, 0);
  switch (currentPIDEditSelection)
  {
    case 1:
      u8g2.print("C1_PH");
      printPID(heaterPIDChan1.Kp_PREHEAT, heaterPIDChan1.Ki_PREHEAT, heaterPIDChan1.Kd_PREHEAT);
      break;
    case 0:
      u8g2.print("C0_PH");
      printPID(heaterPIDChan0.Kp_PREHEAT, heaterPIDChan0.Ki_PREHEAT, heaterPIDChan0.Kd_PREHEAT);
      break;
    case 3:
      u8g2.print("C1_HT");
      printPID(heaterPIDChan1.Kp_SOAK, heaterPIDChan1.Ki_SOAK, heaterPIDChan1.Kd_SOAK);
      break;
    case 2:
      u8g2.print("C0_HT");
      printPID(heaterPIDChan0.Kp_SOAK, heaterPIDChan0.Ki_SOAK, heaterPIDChan0.Kd_SOAK);
      break;
    case 5:
      u8g2.print("C1_RF");
      printPID(heaterPIDChan1.Kp_REFLOW, heaterPIDChan1.Ki_REFLOW, heaterPIDChan1.Kd_REFLOW);
      break;
    case 4:
      u8g2.print("C0_RF");
      printPID(heaterPIDChan0.Kp_REFLOW, heaterPIDChan0.Ki_REFLOW, heaterPIDChan0.Kd_REFLOW);
      break;
  }
  setCursor(6, 4);
  u8g2.print("------");
}

void printError()
{
  // Refresh the screen
  u8g2log.print("\r");
  u8g2.drawLog(0, 5, u8g2log);

  u8g2.updateDisplayArea(0, 0, 128, 6);

  errorTimeout = 1;
}

#endif
