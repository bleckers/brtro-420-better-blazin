#include "Arduino.h"

#define SUPC_BODVDD_ACTION_Pos      3            /**< \brief (SUPC_BODVDD) Action when Threshold Crossed */
#define SUPC_BODVDD_ACTION_Msk      (0x3ul << SUPC_BODVDD_ACTION_Pos)
#define SUPC_BODVDD_ACTION(value)   (SUPC_BODVDD_ACTION_Msk & ((value) << SUPC_BODVDD_ACTION_Pos))

#define SUPC_BODVDD_PSEL_Pos        12           /**< \brief (SUPC_BODVDD) Prescaler Select */
#define SUPC_BODVDD_PSEL_Msk        (0xFul << SUPC_BODVDD_PSEL_Pos)
#define SUPC_BODVDD_PSEL(value)     (SUPC_BODVDD_PSEL_Msk & ((value) << SUPC_BODVDD_PSEL_Pos))

#define BODVDD_ACTCFG_CONTINUOUS 0
#define BODVDD_STDBYCFG_CONTINUOUS 0


void bod_setup(void) {
  uint32_t temp = 0;

  /* Check if module is enabled. */
  if (SUPC->BODVDD.reg & SUPC_BODVDD_ENABLE) {
    SUPC->BODVDD.reg &= ~SUPC_BODVDD_ENABLE;
  }

  /* Convert BOD prescaler, trigger action and mode to a bitmask */
  temp |= SUPC_BODVDD_PSEL(0) | SUPC_BODVDD_ACTION(1) |
          BODVDD_ACTCFG_CONTINUOUS | BODVDD_STDBYCFG_CONTINUOUS;

  temp |= SUPC_BODVDD_HYST;

  SUPC->BODVDD.reg = SUPC_BODVDD_LEVEL(0x2A) | temp;
}

void watchdog_setup(void) {
  //SAMC21 has fixed 1.024KHz OSC into counter instead of feeding from GCLK on SAMD21

  while (WDT->SYNCBUSY.reg);               // Wait for synchronization
  WDT->CONFIG.reg = 0;
  WDT->CTRLA.reg &= ~WDT_CTRLA_WEN;        // Disable window enable
  WDT->CONFIG.reg |= 0x09;                 // Set to timeout after 4 seconds (4096 clock cycles)
  WDT->CTRLA.reg |= WDT_CTRLA_ENABLE;      // Enable the WDT in normal mode
  while (WDT->SYNCBUSY.reg);               // Wait for synchronization
}

void watchdog_feed(void) {

  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;        // Clear the watchdog timer
  while (WDT->SYNCBUSY.reg) ;
}
