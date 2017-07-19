#include <msp430.h> 
#include "lib/lcd.h"
#include "lib/clock.h"
#include "lib/ports.h"
#include "lib/timer.h"
#include "lib/serial.h"
#include "lib/mpu.h"
#include <math.h>

//Para inicializar o MPU e garantir sua precisão serão realizados os seguintes passos:
//1. Retirar o MPU do modo sleep;
//2. Realizar o auto teste;
//3. Realizar a calibração;
//4. Configurar o MPU;
//5. Selecionar a escala;
//6. Realizar as medições.

//Considerações para a coleta de dados:
//Há um intervalo de 33ms entre cada nova coleta (30Hz)
//Nessa configuração cada minuto contêm 2048 (2^11) amostras (4096 bytes ou 32768 bits)

//#define length 1818
#define length 1024

unsigned int controle [10];

int offset_int[6];					//Offset para accel x y z e gyro x y z
int ax,ay,az,gx,gy,gz;				//Valores coletados com offset
int axi,ayi,azi,gxi,gyi,gzi,tpi;	//Valores coletados sem offset
unsigned char a_scale, g_scale;		//Escalas selecionadas
unsigned char reply[14];			//Vetor com as últimas medidas da função mpuRead
unsigned char aux[14]; 				//Para leitura dos registradores
int i;
int data_collect[length];

unsigned char ah,al;  // 8 bits para o lcd - observar mudancas em ax

int main(void) {

__enable_interrupt();
    setupWatchdog();                        // Stop watchdog
    setupPorts();                           // Setup ports
    setupClock();                           // Setup clock
    setupTimerA0();                         // Setup timer
    setupSerial(SYNC, I2C);                 // Reset Serial Interface
    lcdInit();                              // Init LCD: 4-bit mode
    GREEN_DIR |= GREEN_BIT;
    RED_DIR |= RED_BIT;
    RED_LED &= ~RED_BIT;

//Select scales
    a_scale = MPU6050_ACCEL_FS_16;	//2 4 8 16 g
    g_scale = MPU6050_GYRO_FS_250;		//250 500 1000 2000

    for (i=0;i<10;i++){controle[i] = 0;}
    for (i=0;i<14;i++){reply[i]=0x21;}		//!
    for (i=0;i<14;i++){aux[i]=0x21;}		//!

    //Retirar MPU do modo sleep
    lcdWriteStr("Waking MPU");
    mpuSetByte(MPU6050_RA_PWR_MGMT_1, 0x01);
    waitFor(250);

    //Testar comunicacao
    mpuRead_nb(MPU6050_RA_WHO_AM_I,reply,1);
    if(reply[0]==0x68) { lcdGoTo(1,0);	lcdWriteStr("Com Ok");}
    else while(1){};

    //Self-Test
    lcdGoTo(0,0);
    if (mpuSelfTest()) lcdWriteStr("Self Test Ok");
    else{ lcdWriteStr("Erro Self Test");
    	while (1){};
    }
    //Calibragem
    mpuOffset(g_scale,a_scale);
    lcdGoTo(0,0);
    lcdWriteStr("Calibrado");

    //Re-inicializar o MPU
    mpuSetByte(MPU6050_RA_PWR_MGMT_1, 0x80);	//Reset do mpu
    waitFor(250);
    mpuSetByte(MPU6050_RA_PWR_MGMT_1, 0x01);	//Retira do modo sleep e seleciona o clock do PLL do gyro eixo x
    waitFor(250);
    mpuSetByte(MPU6050_RA_CONFIG, 0x03);		//Taxa 1khz Accel=44hz Gyro=42hz
    mpuSetByte(MPU6050_RA_SMPLRT_DIV, 0x04);	//Taxa de amostragem: 200Hz			taxa de amostragem = taxa / (1+SMPLRT_DIV)

    //Selecionar escala
    mpuScales(g_scale,a_scale);

    //Realizar Medições
    lcdGoTo(1, 0);
    lcdWriteStr("ACCEL X: ");

    RED_LED |= RED_BIT;

    for(i=0;i<length;i++){
    	data_collect[i] = mpuAX_DATA();
    	if(i%30==0)
    	{
    	lcdGoTo(1,9);
    	lcdWriteByte(CHARACTER, (data_collect[i]>>12) < 0xA? 0x30 + (data_collect[i]>>12) : 0x37 + (data_collect[i]>>12) );
    	lcdWriteByte(CHARACTER, ((data_collect[i]>>8) & 0x0F) < 0xA? 0x30 + ((data_collect[i]>>8) & 0x0F) : 0x37 + ((data_collect[i]>>8) & 0x0F) );
    	lcdWriteByte(CHARACTER, ((data_collect[i]>>4) & 0x0F) < 0xA? 0x30 + ((data_collect[i]>>8) & 0x0F) : 0x37 + ((data_collect[i]>>8) & 0x0F) );
    	lcdWriteByte(CHARACTER, (data_collect[i] & 0x0F) < 0xA? 0x30 + (data_collect[i] & 0x0F) : 0x37 + ((data_collect[i]>>8) & 0x0F) );
    	}
    	waitFor(33);
    }

    RED_LED &= ~RED_BIT;

    __low_power_mode_0();
}

void mpuDATA_Cal(void){
	//mpuRead(MPU6050_RA_ACCEL_XOUT_H, reply, 14);
	mpuRead_nb(MPU6050_RA_ACCEL_XOUT_H, reply, 14);
	    	axi = (int) ((reply[0] << 8) | reply[1]) ;
	    	ayi = (int) ((reply[2] << 8) | reply[3]) ;
	    	azi = (int) ((reply[4] << 8) | reply[5]) ;
	    	gxi = (int) ((reply[8] << 8) | reply[9]) ;
	    	gyi = (int) ((reply[10] << 8) | reply[11]) ;
	    	gzi = (int) ((reply[12] << 8) | reply[13]) ;

	    	ax = axi - offset_int[1];
	    	ay = ayi - offset_int[1];
	    	az = azi - offset_int[2];
	    	gx = gxi - offset_int[3];
	    	gy = gyi - offset_int[4];
	    	gz = gzi - offset_int[5];

	    	lcdGoTo(1,9);
	    	ah = reply[0] >> 4;
	    	al = reply[1] >> 4;
	    	lcdWriteByte(CHARACTER, ah < 0xA? 0x30 + ah : 0x37 + ah );
	    	lcdWriteByte(CHARACTER, al < 0xA? 0x30 + al : 0x37 + al );

	    	GREEN_LED ^= GREEN_BIT;
}

int mpuAX_DATA(void){

	signed int data;

	mpuRead_nb(MPU6050_RA_ACCEL_XOUT_H, reply, 2);
	axi = (int) ((reply[0] << 8) | reply[1]) ;

	ax = axi - offset_int[0];
	data = ax;

	return data;
}

unsigned char mpuSelfTest(void){
	//unsigned char aux[14]; 					//Para leitura dos registradores
	int gx1, gy1, gz1, ax1, ay1, az1; 			//Valores coletados com self test desabilitado
	int gx2, gy2, gz2, ax2, ay2, az2; 			//Valores coletados com self test desabilitado
	unsigned char st3[6];						//Valores coletados do registrador de self-test
	float stf[6];								//Valores de Factory Trim
	float str[6];								//Alteração em %

//Passo 1: Desabilitar o self test e realizar as leiuras dos accel, gyro e temp
	//As escalas devem estar em 8g e 250
	mpuSetByte(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_8 << 3);
	mpuSetByte(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_250 << 3);
	waitFor(250);

	mpuRead_nb(MPU6050_RA_ACCEL_XOUT_H, aux, 14);
	ax1 = (int) ((aux[0] << 8) | aux[1]) ;
	ay1 = (int) ((aux[2] << 8) | aux[3]) ;
	az1 = (int) ((aux[4] << 8) | aux[5]) ;
	gx1 = (int) ((aux[8] << 8) | aux[9]) ;
	gy1 = (int) ((aux[10] << 8) | aux[11]) ;
	gz1 = (int) ((aux[12] << 8) | aux[13]) ;

//Passo 2: Habilitar o self test e realizar as leiuras dos accel gyro
	//As escalas devem estar em 8g e 250
	mpuSetByte(MPU6050_RA_ACCEL_CONFIG, (0xE0 | MPU6050_ACCEL_FS_8 << 3));
	mpuSetByte(MPU6050_RA_GYRO_CONFIG, (0xE0 | MPU6050_GYRO_FS_250 << 3));
	waitFor(250);

	controle[1] = 4;
	mpuRead_nb(MPU6050_RA_ACCEL_XOUT_H, aux, 14);
	ax2 = (int) ((aux[0] << 8) | aux[1]) ;
	ay2 = (int) ((aux[2] << 8) | aux[3]) ;
	az2 = (int) ((aux[4] << 8) | aux[5]) ;
	gx2 = (int) ((aux[8] << 8) | aux[9]) ;
	gy2 = (int) ((aux[10] << 8) | aux[11]) ;
	gz2 = (int) ((aux[12] << 8) | aux[13]) ;

//Passo 3: Extrair os dados dos registradores de self-test
	mpuRead_nb(MPU6050_RA_SELF_TEST_X, aux, 4);
	st3[0] = (aux[0] >> 3) | (aux[3] & 0x30) >> 4 ;
	st3[1] = (aux[1] >> 3) | (aux[3] & 0x0C) >> 2 ;
	st3[2] = (aux[2] >> 3) | (aux[3] & 0x03) >> 0 ;
	st3[3] = aux[0] & 0x1F ;
	st3[4] = aux[1] & 0x1F ;
	st3[5] = aux[2] & 0x1F ;

//Passo 4: Calcular o Factory Trim
	stf[0] = (4096.0*0.34) * (pow((0.92/0.34) , (((float)st3[0] - 1.0) / 30.0)));
	stf[1] = (4096.0*0.34) * (pow((0.92/0.34) , (((float)st3[1] - 1.0) / 30.0)));
	stf[2] = (4096.0*0.34) * (pow((0.92/0.34) , (((float)st3[2] - 1.0) / 30.0)));
	stf[3] = ( 25.0 * 131.0) * (pow( 1.046 , ((float)st3[3] - 1.0) ));
	stf[4] = (-25.0 * 131.0) * (pow( 1.046 , ((float)st3[4] - 1.0) ));
	stf[5] = ( 25.0 * 131.0) * (pow( 1.046 , ((float)st3[5] - 1.0) ));

	//Caso especial: se o registrador de self test for 0 o factory trim será 0
	if (st3[0] == 0) stf[0] = 0;
	if (st3[1] == 0) stf[1] = 0;
	if (st3[2] == 0) stf[2] = 0;
	if (st3[3] == 0) stf[3] = 0;
	if (st3[4] == 0) stf[4] = 0;
	if (st3[5] == 0) stf[5] = 0;

//Passo 5: Cálculo das Percentagens de Alteração
	str[0] = 100.0 * ((float) (ax2 - ax1) - stf[0]) / stf[0];
	str[1] = 100.0 * ((float) (ay2 - ay1) - stf[1]) / stf[1];
	str[2] = 100.0 * ((float) (az2 - az1) - stf[2]) / stf[2];
	str[3] = 100.0 * ((float) (gx2 - gx1) - stf[3]) / stf[3];
	str[4] = 100.0 * ((float) (gy2 - gy1) - stf[4]) / stf[4];
	str[5] = 100.0 * ((float) (gz2 - gz1) - stf[5]) / stf[5];

//Passo 6: Checar se as percentagens de alteração estão abaixo de 14%
	if (str[0]<14 && str[1]<14 && str[2]<14 && str[3]<14 && str[4]<14 && str[5]<14)
	//if (axr<14 && gxr<14 && gyr<14 && gzr<14)
		 return 1;	//Self-test passou
	else {
		lcdGoTo(1,0);
		if (str[0]>=14) {lcdWriteStr("ax ");}
		if (str[1]>=14) {lcdWriteStr("ay ");}
		if (str[2]>=14) {lcdWriteStr("az ");}
		if (str[3]>=14) {lcdWriteStr("gx ");}
		if (str[4]>=14) {lcdWriteStr("gy ");}
		if (str[5]>=14) {lcdWriteStr("gz");}
		lcdGoTo(0,0);
	return 0;	//Self-test falhou
	}
}

void	mpuScales	(unsigned char gyro, unsigned char accl)
{
	mpuSetByte(MPU6050_RA_GYRO_CONFIG , gyro);
	mpuSetByte(MPU6050_RA_ACCEL_CONFIG , accl);
}

void mpuOffset (unsigned char gyro, unsigned char accl){

	unsigned int j;
	unsigned char aux[14];
	long abx,aby,abz,gbx,gby,gbz;

	abx=aby=abz=gbx=gby=gbz=0;

	mpuSetByte(MPU6050_RA_PWR_MGMT_1, 0x80);	//Reset do mpu
	waitFor(250);
	mpuSetByte(MPU6050_RA_PWR_MGMT_1, 0x01);	//Retira do modo sleep e seleciona o clock do PLL do gyro eixo x
	waitFor(250);

	mpuScales(gyro,accl);
	mpuSetByte(MPU6050_RA_CONFIG, 0x01);		//Seleciona a maior banda possivel para os filtros passa baixo e o giroscópio operara em 1khz
	mpuSetByte(MPU6050_RA_SMPLRT_DIV, 0x00);	//Taxa de amostragem em 1kHz

	//Para otimizar o tempo das amostras será utilizado o data_rdy_int
	//Este bit indica quando os dados de uma leitura foram coletados
	//Será habilitada interrupção para que seja possível coletar os dados antes da próxima leitura

	mpuSetByte(MPU6050_RA_INT_ENABLE, 0x01);	//Habilita interrupção para pooling
	mpuGetByte(MPU6050_RA_INT_STATUS);			//Ler apaga o bit DATA_RDY_INT

	//Serão realizadas 256 leituras
	for (j=0;j<256;j++){
		while((mpuGetByte(MPU6050_RA_INT_STATUS)&1)==0);	//Aguarda a coleta dos dados
		mpuRead_nb(MPU6050_RA_ACCEL_XOUT_H, aux, 14);
		abx += (long) (((unsigned int)aux[0] << 8) | aux[1]) ;
		aby += (long) (((unsigned int)aux[2] << 8) | aux[3]) ;
		abz += (long) (((unsigned int)aux[4] << 8) | aux[5]) ;
		gbx += (long) (((unsigned int)aux[8] << 8) | aux[9]) ;
		gby += (long) (((unsigned int)aux[10] << 8) | aux[11]) ;
		gbz += (long) (((unsigned int)aux[12] << 8) | aux[13]) ;
	}

	//Em seguida calculamos a média das 256 medidas
	offset_int[0] = (int) (abx>>8);
	offset_int[1] = (int) (aby>>8);
	offset_int[2] = (int) (abz>>8);
	offset_int[3] = (int) (gbx>>8);
	offset_int[4] = (int) (gby>>8);
	offset_int[5] = (int) (gbz>>8);
}

