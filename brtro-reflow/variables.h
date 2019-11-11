
#ifndef _VARIABLES_H
#define _VARIABLES_H

#define START_TEMP_MAX 50
#define NUMBER_OF_TEMP_AVERAGES 6
#define NUMBER_OF_CROSSINGS_AVERAGES 2



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

#define MAX_PROFILES 9

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

struct ReflowProfile
{

  //Temp to reach
  int PreHtTemp = 100;
  int HeatTemp = 180;
  int RefTemp = 240;
  int RefKpTemp = 240;
  int CoolTemp = 50;

  //Seconds to try getting to temp
  int PreHtTime = 80;
  int HeatTime = 220;
  int RefTime = 30;
  int RefKpTime = 30;
  int CoolTime = 80;
};

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
  uint8_t tcState;
  ReflowProfile profiles[MAX_PROFILES];
};

void restoreDefaults(StoredVar * storedVar)
{
  storedVar->profile = 1;

  storedVar->tcState = 0;

  storedVar->KpChan0_PREHEAT = PID_KP_PREHEAT_C0;
  storedVar->KiChan0_PREHEAT = PID_KI_PREHEAT_C0;
  storedVar->KdChan0_PREHEAT = PID_KD_PREHEAT_C0;
  storedVar->KpChan1_PREHEAT = PID_KP_PREHEAT_C1;
  storedVar->KiChan1_PREHEAT = PID_KI_PREHEAT_C1;
  storedVar->KdChan1_PREHEAT = PID_KD_PREHEAT_C1;

  storedVar->KpChan0_SOAK = PID_KP_SOAK_C0;
  storedVar->KiChan0_SOAK = PID_KI_SOAK_C0;
  storedVar->KdChan0_SOAK = PID_KD_SOAK_C0;
  storedVar->KpChan1_SOAK = PID_KP_SOAK_C1;
  storedVar->KiChan1_SOAK = PID_KI_SOAK_C1;
  storedVar->KdChan1_SOAK = PID_KD_SOAK_C1;

  storedVar->KpChan0_REFLOW = PID_KP_REFLOW_C0;
  storedVar->KiChan0_REFLOW = PID_KI_REFLOW_C0;
  storedVar->KdChan0_REFLOW = PID_KD_REFLOW_C0;
  storedVar->KpChan1_REFLOW = PID_KP_REFLOW_C1;
  storedVar->KiChan1_REFLOW = PID_KI_REFLOW_C1;
  storedVar->KdChan1_REFLOW = PID_KD_REFLOW_C1;
  storedVar->valid = true;
  storedVar->celsiusMode = true;

  for (int i = 0; i < MAX_PROFILES; i++)
  {

    storedVar->profiles[i].PreHtTemp = 100;
    storedVar->profiles[i].HeatTemp = 180;
    storedVar->profiles[i].RefTemp = 240;
    storedVar->profiles[i].RefKpTemp = 240;
    storedVar->profiles[i].CoolTemp = 60;

    storedVar->profiles[i].PreHtTime = 80;
    storedVar->profiles[i].HeatTime = 180;
    storedVar->profiles[i].RefTime = 60;
    storedVar->profiles[i].RefKpTime = 30;
    storedVar->profiles[i].CoolTime = 80;
  }
}

void editProfileTime(int profileValueBeingEdited, ReflowProfile * profiles, int currentProfile, bool increment)
{
  int value = -1;
  if (increment) value = 1;

  switch (profileValueBeingEdited)
  {
    case PREHT:
      {
        profiles[currentProfile].PreHtTime = profiles[currentProfile].PreHtTime + value;
        if (profiles[currentProfile].PreHtTime < 0) profiles[currentProfile].PreHtTime = 200;
        if (profiles[currentProfile].PreHtTime > 200) profiles[currentProfile].PreHtTime = 0;
        break;
      }
    case HEAT:
      {
        profiles[currentProfile].HeatTime = profiles[currentProfile].HeatTime + value;
        if (profiles[currentProfile].HeatTime < 0) profiles[currentProfile].HeatTime = 200;
        if (profiles[currentProfile].HeatTime > 200) profiles[currentProfile].HeatTime = 0;
        break;
      }
    case REF:
      {
        profiles[currentProfile].RefTime = profiles[currentProfile].RefTime + value;
        if (profiles[currentProfile].RefTime < 0) profiles[currentProfile].RefTime = 200;
        if (profiles[currentProfile].RefTime > 200) profiles[currentProfile].RefTime = 0;
        break;
      }
    case REFKP:
      {
        profiles[currentProfile].RefKpTime = profiles[currentProfile].RefKpTime + value;
        if (profiles[currentProfile].RefKpTime < 0) profiles[currentProfile].RefKpTime = 200;
        if (profiles[currentProfile].RefKpTime > 200) profiles[currentProfile].RefKpTime = 0;
        break;
      }
    case COOL:
      {
        profiles[currentProfile].CoolTime = profiles[currentProfile].CoolTime + value;
        if (profiles[currentProfile].CoolTime < 0) profiles[currentProfile].CoolTime = 200;
        if (profiles[currentProfile].CoolTime > 200) profiles[currentProfile].CoolTime = 0;
        break;
      }
  }
}

void editProfileTemp(int profileValueBeingEdited, ReflowProfile * profiles, int currentProfile, bool increment)
{
  int value = -1;
  if (increment) value = 1;
  switch (profileValueBeingEdited)
  {
    case PREHT:
      {
        profiles[currentProfile].PreHtTemp = profiles[currentProfile].PreHtTemp + value;
        if (profiles[currentProfile].PreHtTemp < 0) profiles[currentProfile].PreHtTemp = 250;
        if (profiles[currentProfile].PreHtTemp > 250) profiles[currentProfile].PreHtTemp = 0;
        break;
      }
    case HEAT:
      {
        profiles[currentProfile].HeatTemp = profiles[currentProfile].HeatTemp + value;
        if (profiles[currentProfile].HeatTemp < 0) profiles[currentProfile].HeatTemp = 250;
        if (profiles[currentProfile].HeatTemp > 250) profiles[currentProfile].HeatTemp = 0;
        break;
      }
    case REF:
      {
        profiles[currentProfile].RefTemp = profiles[currentProfile].RefTemp + value;
        if (profiles[currentProfile].RefTemp < 0) profiles[currentProfile].RefTemp = 250;
        if (profiles[currentProfile].RefTemp > 250) profiles[currentProfile].RefTemp = 0;
        break;
      }
    case REFKP:
      {
        profiles[currentProfile].RefKpTemp = profiles[currentProfile].RefKpTemp + value;
        if (profiles[currentProfile].RefKpTemp < 0) profiles[currentProfile].RefKpTemp = 250;
        if (profiles[currentProfile].RefKpTemp > 250) profiles[currentProfile].RefKpTemp = 0;
        break;
      }
    case COOL:
      {
        profiles[currentProfile].CoolTemp = profiles[currentProfile].CoolTemp + value;
        if (profiles[currentProfile].CoolTemp < 0) profiles[currentProfile].CoolTemp = 250;
        if (profiles[currentProfile].CoolTemp > 250) profiles[currentProfile].CoolTemp = 0;
        break;
      }
  }
}
#endif
