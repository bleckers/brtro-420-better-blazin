#include <Arduino.h>

#ifndef _HELPER_H
#define _HELPER_H

#define SO 0
#define CSU2 1
#define BT3 2
#define BT4 3
#define SCK 4
#define CSU3 5
#define SPK 6
#define OT1 7
#define OT2 8
#define OT3 9
#define OT4 32
#define DI 34
#define ZER_D 35
#define BT1 27
#define BT2 28

#define RW 13
#define DB0 29
#define DB1 16
#define DB2 17
#define DB3 18
#define DB4 19
#define DB5 20
#define DB6 21
#define DB7 22
#define CS1 23
#define CS2 24
#define RST 25
#define E 26

#define BACK BT4
#define OK BT3
#define MINUS BT2
#define PLUS BT1

#define FRONT_HEATER OT1
#define BACK_HEATER OT2
#define EXHAUST OT3
#define CONVECTION    OT4

//TC1 is front (by default)
//TC0 is back  (by default)

//OT4 is convection
//OT3 is fan
//OT2 is front
//OT1 is back

//States
#define PREHT 0
#define HEAT 1
#define REF 2
#define REFKP 3
#define COOL 4
#define TUNE 5
#define IDLEM 6
#define NONE 7

#define BOOT_MAGIC_ADDRESS (0x20007FFCul)
#define BOOT_MAGIC_VALUE (*((volatile uint32_t *)BOOT_MAGIC_ADDRESS))
#define BOOT_MAGIC_BOOTLOADER_ENABLE 0x32

void setOutputs()
{
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
}

void buzzer(int pin)
{
  pinMode(pin, OUTPUT);
  tone(pin, 700);
  delay(200);
  noTone(pin);

  pinMode(pin, INPUT);
}

int intLen(int n) {
  unsigned int number_of_digits = 0;

  if (n < 0) number_of_digits++;

  do {
    ++number_of_digits;
    n /= 10;
  } while (n);

  return number_of_digits;
}

double cToF(double celsius)
{
  return ((double)1.8 * celsius) + 32;
}

double avg(double *array, uint8_t count)
{
  double sum = 0;

  for (uint8_t i = 0; i < count; i++)
  {
    sum += array[i];
  }

  return sum / count;
}

unsigned long avg(volatile unsigned long *array, uint8_t count)
{
  unsigned long sum = 0;

  for (uint8_t i = 0; i < count; i++)
  {
    sum += array[i];
  }

  return sum / count;
}

#endif
