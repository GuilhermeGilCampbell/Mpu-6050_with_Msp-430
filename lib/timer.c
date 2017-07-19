#include <msp430.h>
#include "timer.h"

void setupWatchdog() {
    // Watchdog configuration
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
}

void setupTimerA0() {
  // Timer configuration
  TA0CTL = TASSEL__ACLK      |              // Select ACLK as clock source
           ID__1             |              // Set clock divider to 1.
           MC__STOP          |              // Setup but do not count
           TACLR             |              // Clear timer
           TAIE              |              // Enable Interrupts from timer
           0;

  // Timer A0 - CCR0 configuration
  TA0CCR0  = 32767;                          // Count to 1s @32kHz

}

void waitFor(unsigned int time_ms)
{
  // Configure timer A0 and start it.
  TA0CCR0 = time_ms << 5;                   // 1 sec = 1024 ms (rounded up)
  TA0CTL  |= MC__UP | TACLR;                // Count up and clear timer

  // Locks, waiting for the timer.
  __low_power_mode_0();
  //while (!(TA0CCTL0 & CCIFG)) {};           // Wait until the end
  //TA0CCTL0 &= ~CCIFG;                       // Clear flag and
  //TA0CTL &= ~0x30;                          // Stop timer
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void ISR_TIMER0 (void) {
  switch (__even_in_range(TA0IV,14)) {
    case 0x0 : break;   // No interruption
    case 0x2 : break;   // CCR1
    case 0x4 : break;   // CCR2
    case 0x6 : break;   // CCR3
    case 0x8 : break;   // CCR4
    case 0xA : break;   // CCR5
    case 0xC : break;   // CCR6
    case 0xE :          // Overflow
      // Stop the timer
      TA0CTL &= ~0x30;
      // And exit low power mode
      __low_power_mode_off_on_exit();
      break;
    default  : break;
  }
}

