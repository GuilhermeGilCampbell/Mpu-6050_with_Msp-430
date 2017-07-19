#include <msp430.h>
#include "ports.h"

void setupPorts() {

    // Configure ports
    // Remove warning of uninitialized ports
    // This will set all ports to input with a pull-down resistor
    PADIR = 0x0000; PAREN = 0xFFFF; PAOUT = 0x0000; PASEL = 0x0000;
    PBDIR = 0x0000; PBREN = 0xFFFF; PBOUT = 0x0000; PBSEL = 0x0000;
    PCDIR = 0x0000; PCREN = 0xFFFF; PCOUT = 0x0000; PCSEL = 0x0000;
    PDDIR = 0x0000; PDREN = 0xFFFF; PDOUT = 0x0000; PDSEL = 0x0000;

    // Set all LCD ports as outputs
    D4_DIR |= D4_BIT;
    D5_DIR |= D5_BIT;
    D6_DIR |= D6_BIT;
    D7_DIR |= D7_BIT;
    RS_DIR |= RS_BIT;
    EN_DIR |= EN_BIT;

    // Configure crystal ports
    P5SEL |= BIT2 | BIT3 | BIT4 | BIT5;     // Configure P5 to use Crystals

    // Switch 1
    S1DIR &= ~S1BIT;                        // Input direction
    S1REN |=  S1BIT;                        // Enable resistor
    S1OUT |=  S1BIT;                        // Pull-up

    // Switch 2
    S2DIR &= ~S2BIT;                        // Input direction
    S2REN |=  S2BIT;                        // Enable resistor
    S2OUT |=  S2BIT;                        // Pull-up

    // LEDs
    RED_DIR   |=  RED_BIT;                  // Red   LED : output
    RED_LED   &= ~RED_BIT;                  // Red   LED : off

    GREEN_DIR |=  GREEN_BIT;                // Green LED : output
    GREEN_LED &= ~GREEN_BIT;                // Green LED : off

    // Serial communication
    SDA_SEL |=  SDA_BIT;                     // Use dedicated module
    SDA_REN &= ~SDA_BIT;                     // Resistor enable
    SDA_OUT |=  SDA_BIT;                     // Pull-up

    SCL_SEL |=  SCL_BIT;                     // Use dedicated module
    SCL_REN &= ~SCL_BIT;                     // Resistor enable
    SCL_OUT |=  SCL_BIT;                     // Pull-up

}

void portsWriteLCDBUS(unsigned char data) {

	// Write data to output pins
	D4_OUT = (D4_OUT & (~D4_BIT)) | ((data & BIT0) ? D4_BIT : 0);
	D5_OUT = (D5_OUT & (~D5_BIT)) | ((data & BIT1) ? D5_BIT : 0);
	D6_OUT = (D6_OUT & (~D6_BIT)) | ((data & BIT2) ? D6_BIT : 0);
	D7_OUT = (D7_OUT & (~D7_BIT)) | ((data & BIT3) ? D7_BIT : 0);
}

void setLED(unsigned char color, unsigned char action) {
    if(color == GREEN) {
        // Green LED
        switch(action) {
            case ON :
                GREEN_LED |=  GREEN_BIT;
                break;
            case OFF :
                GREEN_LED &= ~GREEN_BIT;
                break;
            case TOGGLE :
                GREEN_LED ^=  GREEN_BIT;
                break;
            default :
                GREEN_LED &= ~GREEN_BIT;
        }
    }
    if (color == RED) {
        // Red LED
        switch(action) {
            case ON :
                RED_LED |=  RED_BIT;
                break;
            case OFF :
                RED_LED &= ~RED_BIT;
                break;
            case TOGGLE :
                RED_LED ^=  RED_BIT;
                break;
            default :
                RED_LED &= ~RED_BIT;
        }
    }
}

unsigned char readButton(unsigned char button) {
    if (button) // S2
        return (S2 & S2BIT) ? 0 : 1;
    else
        return (S1 & S1BIT) ? 0 : 1;
}
