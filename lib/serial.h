#ifndef SERIAL_H_
#define SERIAL_H_

#define ASYNC           00
#define SYNC            01

#define UART            00
#define THREEPIN_SPI    01
#define FOURPIN_SPI     02
#define I2C             03

#define SLAVE           00
#define MASTER          01

#define TRANSMITTER     00
#define RECEIVER        01

#define WAIT_ACK        (UCB0CTL1 & UCTXSTT)

struct {
    unsigned char * data;
    unsigned char count;
    unsigned char index;
    enum {sendBurst,sendAndRestart} mode;
} TX;

struct {
    unsigned char * data;
    unsigned char count;
    unsigned char index;
} RX;

// Setup communication interface (requires reset)
void setupSerial(   unsigned char sync,
                    unsigned char protocol);

// Setup serial interface (no reset needed)
void serialSetMode( unsigned char mode,
                    unsigned char direction);

// Basic communication actions
void serialStart(unsigned char deviceAddr); // Start
void serialStop();                          // Stop
void serialRestart();                       // Restart (same as start)

// Send a package of data bytes
void         serialSendData(unsigned char  deviceAddr,
                            unsigned char *dataPtr,
                            unsigned char  count);

// Send only one byte
void         serialSendByte(unsigned char  deviceAddr,
                            unsigned char  data);

// Receive a package of data bytes
void         serialGetData( unsigned char deviceAddr,
                            unsigned char from,
                            unsigned char * to,
                            unsigned char count);

// Receive only one byte
unsigned char serialGetByte(unsigned char deviceAddr,
                            unsigned char from);

#endif /* SERIAL_H_ */
