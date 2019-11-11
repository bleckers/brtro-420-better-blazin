#include "helper.h"
#include "variables.h"

#ifndef _SERIAL_H
#define _SERIAL_H

void serialPrintCurrentState(int currentState)
{
  switch (currentState)
  {
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
}

void serialPrintTuningDoneReflow(PID3_t heaterPIDChan0, PID3_t heaterPIDChan1)
{
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
}

void serialPrintTuningDoneHeat(PID3_t heaterPIDChan0, PID3_t heaterPIDChan1)
{
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
}

void serialPrintTuningDonePreheat(PID3_t heaterPIDChan0, PID3_t heaterPIDChan1)
{

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
}

void serialPrintProfile(int tcState, int currentProfile, ReflowProfile * profiles, PID3_t heaterPIDChan0, PID3_t heaterPIDChan1, bool celsiusMode) {
  Serial.println("");
  Serial.println ("Reflow Started");
  if (tcState == 0)
  {
    Serial.println("Back == TC0, Front == TC1");
  } else if (tcState == 1)
  {
    Serial.println("Back == TC1, Front == TC0");
  } else {
    Serial.println("Back == AVG, Front == AVG");
  }

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
}

#endif
