#include <msp430.h>
#include "mpu.h"
#include "serial.h"
#include "timer.h"

unsigned char mpuGetByte(unsigned char addr);
unsigned char mpuRead   (unsigned char addr, unsigned char * data, unsigned char length);
void          mpuSetByte(unsigned char addr, unsigned char data);
void          mpuWrite  (unsigned char addr, unsigned char * data, unsigned char length);
void		  mpuScales	(unsigned char g_scale, unsigned char a_scale);

void mpuSetByte(unsigned char addr, unsigned char data)
{
    unsigned char dataPtr[2];
    dataPtr[0] = addr;
    dataPtr[1] = data;

    serialSendData(MPU6050_DEFAULT_ADDRESS, dataPtr , 2);
}

unsigned char mpuGetByte(unsigned char addr)
{
    return serialGetByte(MPU6050_DEFAULT_ADDRESS, addr);
}


unsigned char mpuRead(unsigned char addr, unsigned char * data, unsigned char length)
{
    UCB0IE = 0x00;                          // This routine won't use
                                            // interruptions, disable them
    if (UCB0STAT & UCBBUSY )                // If the bus is occupied,
        return MPU6050_BUS_BUSY;            // return bus occupied error code

    UCB0I2CSA = MPU6050_DEFAULT_ADDRESS;    // Set slave address
    UCB0CTL1 |= UCTR + UCTXSTT;             // Start the transaction

    while (!(UCB0IFG & UCTXIFG));           // Wait until TX buffer is empty
    if(UCB0STAT & UCNACKIFG)                // If there was no acknowledgment
        return MPU6050_NO_ONE_LISTENING;    // return an error

    UCB0TXBUF = addr;                       // Send register address and
    UCB0IFG &= ~UCTXIFG;                    // clear USCI_B0 TX int flag

    while (!(UCB0IFG & UCTXIFG));           // Wait until TX buffer is empty
    if(UCB0STAT & UCNACKIFG)                // If the message was not understood
        return MPU6050_REG_OUT_OF_RANGE;    // by the slave, return an error

    UCB0CTL1 &= ~UCTR;                      // then, restart transaction
    UCB0CTL1 |= UCTXSTT;                    // as master receiver
    UCB0IFG &= ~UCTXIFG;                    // Clear USCI_B0 TX int flag

    while (UCB0CTL1 & UCTXSTT);             // Wait for slave acknowledgment

    unsigned char i,j=0;
    for(i=length-1;i>0;i--)                 // And start receiving data
    {
        while (!(UCB0IFG  & UCRXIFG));      // Whenever RX buffer is full,
        //UCB0IFG &= ~UCRXIFG;                // Clear USCI_B0 RX int flag
        data[j++] = UCB0RXBUF;                // save received data
    }

    while (!(UCB0IFG & UCRXIFG));           // Upon reception of last byte,
    UCB0CTL1 |=  UCTXSTP;                   // stop transaction and
    data[length-1]   =  UCB0RXBUF;                 // read last byte received
//    UCB0IFG  &= ~UCTXIFG;                    // Clear USCI_B0 TX int flag
//    UCB0IFG  &= ~UCRXIFG;                    // Clear USCI_B0 TX int flag
    return MPU6050_SUCESS;
}

void mpuRead_v2(unsigned char addr, unsigned char * data, unsigned char length)
{
	unsigned char counter=1;
	while(counter!=MPU6050_SUCESS){
		counter=mpuRead(addr,data,length);
}
}

void mpuRead_nb(unsigned char addr, unsigned char * data, unsigned char length)
{
	unsigned char i;
	for (i=length;i>0;i--){
		data[length-i]=mpuGetByte(addr+(length-i));
	}
}


