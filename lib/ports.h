#ifndef PORTS_H_
#define PORTS_H_

// Pins definitions for LCD
// D4 - 11 - P6.0
// D5 - 12 - P6.1
// D6 - 13 - P6.2
// D7 - 14 - P6.3
// EN - 06 - P6.5
// RS - 04 - P6.6

#define RS_SEL  P6SEL
#define RS_DIR  P6DIR
#define RS_OUT  P6OUT
#define RS_BIT  BIT6

#define EN_SEL  P6SEL
#define EN_DIR  P6DIR
#define EN_OUT  P6OUT
#define EN_BIT  BIT5

#define D4_SEL  P6SEL
#define D4_DIR  P6DIR
#define D4_OUT  P6OUT
#define D4_BIT  BIT0

#define D5_SEL  P6SEL
#define D5_DIR  P6DIR
#define D5_OUT  P6OUT
#define D5_BIT  BIT1

#define D6_SEL  P6SEL
#define D6_DIR  P6DIR
#define D6_OUT  P6OUT
#define D6_BIT  BIT2

#define D7_SEL  P6SEL
#define D7_DIR  P6DIR
#define D7_OUT  P6OUT
#define D7_BIT  BIT3


// Switches S1 and S2
// S1 - P2.1
// S2 - P1.1
#define S1      P2IN
#define S1BIT   BIT1
#define S1DIR   P2DIR
#define S1SEL   P2SEL
#define S1OUT   P2OUT
#define S1REN   P2REN

#define S2      P1IN
#define S2BIT   BIT1
#define S2DIR   P1DIR
#define S2SEL   P1SEL
#define S2OUT   P1OUT
#define S2REN   P1REN

// LEDs
#define RED   0
#define GREEN 1
#define OFF   0
#define ON    1

// Red LED   - P1.0
#define RED_LED     P1OUT
#define RED_BIT     BIT0
#define RED_DIR     P1DIR
#define RED_SEL     P1SEL
#define RED_REN     P1REN
// Green LED - P4.7
#define GREEN_LED   P4OUT
#define GREEN_BIT   BIT7
#define GREEN_DIR   P4DIR
#define GREEN_SEL   P4SEL
#define GREEN_REN   P4REN

// Serial communication
#define SDA_SEL     P3SEL
#define SDA_BIT     BIT0
#define SDA_REN     P3REN
#define SDA_OUT     P3OUT

#define SCL_SEL     P3SEL
#define SCL_BIT     BIT1
#define SCL_REN     P3REN
#define SCL_OUT     P3OUT

void setupPorts();
void portsWriteLCDBUS(unsigned char data);
void setLED(unsigned char color,unsigned char state);
unsigned char readButton(unsigned char button);

#endif /* PORTS_H_ */
