#include <msp430.h>
#include "clock.h"

void setupClock () {
    // Unified clock system
    UCSCTL0 = 0x00;                         // Let FLL manage DCO and MOD
    UCSCTL1 = DCORSEL_5;                    // Select DCO range to 16MHz
    UCSCTL2 = FLLD__16        |             // Set FLL dividers
              FLLN(32);
    UCSCTL3 = SELREF__XT1CLK  |             // Use Crystal 1 Oscillator for better precision
              FLLREFDIV__1;                 // FLL input divider to 1
    UCSCTL4 = SELA__XT1CLK    |             // ACLK  = Crystal 1, =>     32.768 Hz
              SELS__DCOCLKDIV |             // SMCLK = DCO/FLLD   =>  1.048.576 Hz
              SELM__DCOCLK;                 // MCLK  = DCO        => 16.777.216 Hz
    UCSCTL5 = DIVPA__1        |             // Output dividers to 1
              DIVA__1         |             // ACLK  divider 1
              DIVS__1         |             // SMCLK divider 1
              DIVM__1;                      // MCLK  divider 1

    UCSCTL6 = XT2DRIVE_3      |             // XT2 and XT1 drive can be
              XT1DRIVE_3      |             // lowered to 0 (default is 3)
              XCAP_3;                       // This is the default (3).
              //XT1OFF    = 0               // Turns on XT1 and XT2
              //XT2OFF    = 0
              //XT1BYPASS = 0
              //XT2BYPASS = 0
              //XTS       = 0

    UCSCTL7 = 0;                            // Clear XT2,XT1,DCO fault flags

    UCSCTL8 = SMCLKREQEN      |             // Enable conditinal requests for
               MCLKREQEN      |             // SMCLK, MCLK and ACLK
               ACLKREQEN;

    while (SFRIFG1 & OFIFG) {               // Test oscillator fault flag
        UCSCTL7 = 0;                        // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                  // Clear fault flags
    };


}
