#include <msp430.h> 

//////////////////////////////////////////////////////////////////////////////////////////

#define MPU6050_DEFAULT_ADDRESS         0x68

// Mpu Registers
#define MPU6050_RA_PWR_MGMT_1           0x6B
#define MPU6050_RA_SMPLRT_DIV           0x19
#define MPU6050_RA_CONFIG               0x1A
#define MPU6050_RA_GYRO_CONFIG          0x1B
#define MPU6050_RA_ACCEL_CONFIG         0x1C
#define MPU6050_RA_ACCEL_XOUT_H         0x3B
#define MPU6050_RA_WHO_AM_I             0x75

// Scales
// Output(degrees/second) = Output(signed integer) * max_range/32767
#define MPU6050_GYRO_FS_250             0x00
#define MPU6050_GYRO_FS_500             0x01
#define MPU6050_GYRO_FS_1000            0x02
#define MPU6050_GYRO_FS_2000            0x03

// Output(g) = Output(signed integer) * max_range/32767
#define MPU6050_ACCEL_FS_2              0x00
#define MPU6050_ACCEL_FS_4              0x01
#define MPU6050_ACCEL_FS_8              0x02
#define MPU6050_ACCEL_FS_16             0x03


/////////////////////////////////////////////////////////////////////////////////////////

#define SLAVE           00
#define MASTER          01

#define TRANSMITTER     00
#define RECEIVER        01

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

//Functions	////////////////////////////////////////////////////////////////////////////////////////////////////////
void 	setupPorts			(void);
void 	setupClock 			(void);
void 	setupTimerA0		(void);
void	waitFor				(unsigned int time_ms);
void 	setupSerial			(void);
void 	serialSetMode		(unsigned char mode, unsigned char direction);
inline void serialStart		(unsigned char addr);
void 	serialSendData		(unsigned char deviceAddr, unsigned char * dataPtr, unsigned char count);
void 	serialGetData		(unsigned char deviceAddr, unsigned char from,unsigned char * to, unsigned char count);
unsigned char serialGetByte	(unsigned char deviceAddr, unsigned char from);
void 	mpuSetByte			(unsigned char addr, unsigned char data);
unsigned char mpuGetByte	(unsigned char addr);
void 	mpuRead_nb			(unsigned char addr, unsigned char * data, unsigned char length);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float			ACCEL[3],GYRO[3],TEMP;		//converted value

int main(void) {

	unsigned char 	reply[14];					//Vetor com as últimas medidas da função mpuRead
	int 			ax,ay,az,gx,gy,gz,temp;		//signed int value
	unsigned char a_scale, g_scale;
	__enable_interrupt();
    	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer
        setupPorts();					// Setup ports
        setupClock();					// Setup clock
        setupTimerA0();					// Setup timer
        setupSerial();					// Reset Serial Interface

        //Desired Scale
        a_scale = MPU6050_ACCEL_FS_16;		//2, 4, 8 or 16 g
        g_scale = MPU6050_GYRO_FS_250;		//250, 500, 1000 or 2000 degrees/second

        //Wake up MPU
        mpuSetByte(MPU6050_RA_PWR_MGMT_1, 0x01);
        waitFor(250);

        //Test Comunication
        mpuRead_nb(MPU6050_RA_WHO_AM_I,reply,1);
        if(reply[0]==MPU6050_DEFAULT_ADDRESS){}
        else while(1){P4OUT |= BIT7;};			//If com test fails, green led on

        //Set scales
        mpuSetByte(MPU6050_RA_GYRO_CONFIG , g_scale<<3);
       	mpuSetByte(MPU6050_RA_ACCEL_CONFIG , a_scale<<3);

        //Get one full reading from accelerometers, giroscope and temperature
       	mpuRead_nb(MPU6050_RA_ACCEL_XOUT_H, reply, 14);
       	ax = (int) ((reply[0] << 8) | reply[1]) ;
        ay = (int) ((reply[2] << 8) | reply[3]) ;
       	az = (int) ((reply[4] << 8) | reply[5]) ;
       	temp = (int) ((reply[6] << 8) | reply[7]) ;
       	gx = (int) ((reply[8] << 8)	| reply[9]) ;
       	gy = (int) ((reply[10] << 8) | reply[11]) ;
       	gz = (int) ((reply[12] << 8) | reply[13]) ;

       	//Data conversion to it's apropriate unit
       	ACCEL[0] = (float) ax*(2<<a_scale)/32767;
       	ACCEL[1] = (float) ay*(2<<a_scale)/32767;
       	ACCEL[2] = (float) az*(2<<a_scale)/32767;
       	TEMP = (float) temp/340 + 36.53;
       	GYRO[0] = (float) gx*(250<<g_scale)/32767;
       	GYRO[1] = (float) gy*(250<<g_scale)/32767;
       	GYRO[2] = (float) gz*(250<<g_scale)/32767;


       	P1OUT = BIT0;	//Red led on after readings
	return 0;
}
;
// Ports Setup ///////////////////////////////////////////////////////////////////////////////////////////////////////
void setupPorts (void){

	// Remove warning of uninitialized ports
	// This will set all ports to input with a pull-down resistor
	    PADIR = 0x0000; PAREN = 0xFFFF; PAOUT = 0x0000; PASEL = 0x0000;
	    PBDIR = 0x0000; PBREN = 0xFFFF; PBOUT = 0x0000; PBSEL = 0x0000;
	    PCDIR = 0x0000; PCREN = 0xFFFF; PCOUT = 0x0000; PCSEL = 0x0000;
	    PDDIR = 0x0000; PDREN = 0xFFFF; PDOUT = 0x0000; PDSEL = 0x0000;

	// Serial communication
	    P3SEL |=  BIT0 | BIT1;                     // Use dedicated module
	    P3REN &= ~BIT0;                     // Resistor enable
	    P3REN &= ~BIT1;
	    P3OUT |=  BIT0 | BIT1;                     // Pull-up

	// LEDs
	    P1DIR   |=  BIT0;                  // Red   LED : output
		P1OUT   &= ~BIT0;                  // Red   LED : off

	    P4DIR |=  BIT7;                // Green LED : output
	    P4OUT &= ~BIT7;                // Green LED : off
}


// Clock Setup ///////////////////////////////////////////////////////////////////////////////////////////////////////
void setupClock (void) {
    // Unified clock system
    UCSCTL0 = 0x00;                         // Let FLL manage DCO and MOD
    UCSCTL1 = DCORSEL_5;                    // Select DCO range to 16MHz
    UCSCTL2 = FLLD__16        |             // Set FLL dividers
              31;
    UCSCTL3 = SELREF__XT1CLK  |             // Use Crystal 1 Oscillator for better precision
              FLLREFDIV__1;                 // FLL input divider to 1
    UCSCTL4 = SELA__XT1CLK    |             // ACLK  = Crystal 1, =>     32.768 Hz
              SELS__DCOCLKDIV |             // SMCLK = DCO/FLLD   =>  1.048.576 Hz
              SELM__DCOCLK;                 // MCLK  = DCO        => 16.777.216 Hz
    UCSCTL5 = DIVPA__1        |             // Output dividers to 1
              DIVA__1         |             // ACLK  divider 1
              DIVS__1         |             // SMCLK divider 1
              DIVM__1;                      // MCLK  divider 1

    UCSCTL7 = 0;                            // Clear XT2,XT1,DCO fault flags

}

//Timer Setup ///////////////////////////////////////////////////////////////////////////////////////////////////////
void setupTimerA0(void) {
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

//Serial Setup ///////////////////////////////////////////////////////////////////////////////////////////////////////
void setupSerial(void)
{
    // UCS0 Module configuration
    // First Reset module
    UCB0CTL1 |= UCSWRST;

    // Protocol Synchronous I2C (only one supported at this time)
    UCB0CTL0 = UCSYNC		|		// Syncronous mode enabled
               UCMODE_3		|		// I2C mode
               UCMST; // Master is the default (should remove this)

    // Set Own Address (and reset UCGCEN, i.e., don't respond to a general call)
    UCB0I2COA = 0x42;

    // Set prescaler SMCLK @1M/10 = 100k
    UCB0BRW = 10;

    // Select SMCLK as clock source and clear reset
    UCB0CTL1 = UCSSEL__SMCLK    |	// CLK source SMCLK
               UCTR ;				// Transmiter mode
    								// UCSWRST cleared
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

//Functions for serial comunication //////////////////////////////////////////////////////////////////////////////////

// Transmiter or receiver
void serialSetMode( unsigned char mode, unsigned char direction)
{
    UCB0CTL0 |= (mode      == MASTER     )? UCMST : 0;
    UCB0CTL1 |= (direction == TRANSMITTER)? UCTR  : 0;
}

// Start
inline void serialStart(unsigned char addr)
{
    // Set slave address
    UCB0I2CSA = addr;

    // Start Transmission
    UCB0CTL1 |= UCTXSTT;
}

// Send "count" bytes of "dataPtr" to device with adress "device addr"
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

// Get "count" bytes from device ("deviceAddr") starting at "from" and write it on "to"
void serialGetData(unsigned char deviceAddr, unsigned char from,
                   unsigned char * to, unsigned char count)
{
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

//Return data  from "from" of the devise "deviceAddr"
unsigned char serialGetByte(unsigned char deviceAddr, unsigned char from)
{
    unsigned char reply;
    serialGetData(deviceAddr, from, &reply, 1);
    return reply;
}

//Write "data" at the internal register "addr" of mpu
void mpuSetByte(unsigned char addr, unsigned char data)
{
    unsigned char dataPtr[2];
    dataPtr[0] = addr;
    dataPtr[1] = data;

    serialSendData(MPU6050_DEFAULT_ADDRESS, dataPtr , 2);
}

//Return data from internal register "addr" of mpu
unsigned char mpuGetByte(unsigned char addr)
{
    return serialGetByte(MPU6050_DEFAULT_ADDRESS, addr);
}

//Read "length" times staring at "addr" from mpu and write it on "data"
void mpuRead_nb(unsigned char addr, unsigned char * data, unsigned char length)
{
	unsigned char i;
	for (i=length;i>0;i--){
		data[length-i]=mpuGetByte(addr+(length-i));
	}
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
                        while (UCB0CTL1 & UCTXSTT);   // byte, then wait for acknowledge
                        UCB0CTL1 |=UCTXSTP; // and send a stop request while
                    }                       // receiving first byte
                }
            }
            break;
        default  : break;
    }
}
//


