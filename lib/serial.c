#include <msp430.h>
#include "serial.h"
#include "ports.h" // for setLED

void setupSerial(unsigned char sync, unsigned char protocol)
{
    // UCS0 Module configuration
    // First Reset module
    UCB0CTL1 |= UCSWRST;

    // Protocol Synchronous I2C (only one supported at this time)
    UCB0CTL0 = ((sync     == SYNC)? UCSYNC   : 0) |
               ((protocol == I2C )? UCMODE_3 : 0) |
               UCMST; // Master is the default (should remove this)

    // Set Own Address (and reset UCGCEN, i.e., don't respond to a general call)
    UCB0I2COA = 0x42;

    // Set prescaler SMCLK @1M/10 = 100k
    UCB0BRW = 10;

    // Select SMCLK as clock source and clear reset
    UCB0CTL1 = UCSSEL__SMCLK    |
               UCTR ;
    // Transmitter is the default (should remove this)

    // Enable module interrupts
    UCB0IE =  UCNACKIE          | // Not-acknowledge
              UCALIE            | // Arbitration lost
              UCSTPIE           | // STOP condition
              UCSTTIE           | // START condition
              UCTXIE            | // TX buffer is ready
              UCRXIE            | // Received something (RX buffer is ready)
              0;                  // TX and RX flags are automatically disabled

}

void serialSetMode( unsigned char mode, unsigned char direction)
{
    UCB0CTL0 |= (mode      == MASTER     )? UCMST : 0;
    UCB0CTL1 |= (direction == TRANSMITTER)? UCTR  : 0;
}

// Shouldn't be needed
inline void serialStop()
{
    // Send stop bit
    UCB0CTL1 |= UCTXSTP;
}

inline void serialStart(unsigned char addr)
{
    // Set slave address
    UCB0I2CSA = addr;

    // Start Transmission
    UCB0CTL1 |= UCTXSTT;
}

void serialRestart() {
    // Use the same slave address as before and
    // restart communication
    UCB0CTL1 |= UCTXSTT;
}

void serialSendData(unsigned char deviceAddr, unsigned char * dataPtr, unsigned char count)
{
    // Setup serial interface (without reseting)
    serialSetMode(MASTER, TRANSMITTER);
    UCB0IE = 0x3F; // Enable interrupts

    UCB0IE = 0x3F;
    TX.data  = dataPtr;
    TX.index = 0;
    TX.count = count;
    TX.mode  = sendBurst;

    serialStart(deviceAddr);
    __low_power_mode_0();
}

void serialSendWord(unsigned char  deviceAddr, unsigned char  data)
{
    // A simplified version of serialSendData
    serialSendData(deviceAddr, &data, 1);
}

void serialGetData(unsigned char deviceAddr, unsigned char from,
                   unsigned char * to, unsigned char count) {

    // Setup serial interface (without reseting)
    serialSetMode(MASTER, TRANSMITTER);
    UCB0IE = 0x3F; // Enable interrupts

    // Assemble TX object
    TX.data  = &from;
    TX.count = 1;
    TX.index = 0;
    TX.mode  = sendAndRestart;

    // Assemble RX object
    RX.data  = to;
    RX.count = count;
    RX.index = 0;

    // Start communication
    serialStart(deviceAddr);
    __low_power_mode_0();

}

// A simplified version of serialGetData
unsigned char serialGetByte(unsigned char deviceAddr, unsigned char from) {
    unsigned char reply;
    serialGetData(deviceAddr, from, &reply, 1);
    return reply;
}

#pragma vector=USCI_B0_VECTOR
__interrupt void ISR_USCI_B0 (void) {
    switch (__even_in_range(UCB0IV,12)) {
        case 0x0 : break;                   // -> No interrupt pending
        case 0x2 : break;                   // -> Arbitration lost
        case 0x4 :                          // -> Not acknowledgment
            UCB0CTL1 |= UCTXSTP;            // Stop transmission because
            __low_power_mode_off_on_exit(); // there is no one listening
            break;
        case 0x6 : break;                   // -> Start condition received
        case 0x8 : break;                   // -> Stop condition received
        case 0xA :                          // -> Data received (RXBUF full)
            RX.data[RX.index++] = UCB0RXBUF;
            if (RX.index == (RX.count-1)) {
                UCB0CTL1 = UCTXSTP;         // Request end of transmission while
            }                               // receiving the last byte
            if (RX.index == RX.count) {
                __low_power_mode_off_on_exit();
            }
            break;

        case 0xC :                          // -> Ready to send (TXBUF empty)
            if (TX.index != TX.count) {
                UCB0TXBUF =                 // Load TX buffer while
                      TX.data[TX.index++];  // there is data to be transfered
            } else {
                if (TX.mode == sendBurst) { // If previous transmission was the last
                    UCB0CTL1 |= UCTXSTP;    // Simply stop transmission and exit
                    __low_power_mode_off_on_exit();
                }
                if (TX.mode == sendAndRestart) {
                    UCB0CTL1 &= ~UCTR;      // Set master receiver
                    UCB0CTL1 |= UCTXSTT;    // Restart communication
                    if (RX.count == 1) {    // If we expect to receive a single
                        while (WAIT_ACK);   // byte, then wait for acknowledge
                        UCB0CTL1 |=UCTXSTP; // and send a stop request while
                    }                       // receiving first byte
                }
            }
            break;
        default  : break;
    }
}
