#include "variable.h"

long *da[4] = {0 , 0 , 0 , 0};
int	 da_type[4] = {0 , 0 , 0 , 0} , da_data[4] ={0 ,0 ,0 ,0};
float da_scale[4] = {5., 5., 5., 5.}, da_mid[4] = {0., 0., 0., 0.}, da_temp[4] = {0., 0., 0., 0.};

void InitSerialDac(void)
{
	SpidRegs.SPICCR.bit.SPISWRESET = 0;
	SpidRegs.SPICCR.all = 0x0047;			//char length 8bit		// Clock polarity = 1

	SpidRegs.SPICTL.all = 0x000E;			// Master mode, Enable transmission, Phase = 0
	SpidRegs.SPISTS.all = 0x0000;
	SpidRegs.SPIBRR = 0x0005;				//LSPCLK(150MHz)/6 = 25MHz
    SpidRegs.SPIPRI.bit.FREE = 1;

    SpidRegs.SPIFFTX.all = 0xC008;			// Enable FIFO's, set TX FIFO level to 8
    SpidRegs.SPIFFRX.all = 0x001F;			// Set RX FIFO level to 8
    SpidRegs.SPIFFCT.all = 0x00;
    SpidRegs.SPIPRI.all = 0x0010;

    SpidRegs.SPICCR.bit.SPISWRESET = 1;

    SpidRegs.SPIFFTX.bit.TXFIFO = 1;
    SpidRegs.SPIFFRX.bit.RXFIFORESET = 1;

	GpioDataRegs.GPBSET.bit.GPIO51 = 1;
	while(SpidRegs.SPIFFTX.bit.TXFFST);
	asm(" nop ");
	GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;
}

void SerialDacOut(void)
{
	da_temp[0] = da_type[0] ? (int)(*da[0]) : *(float *)(da[0]);
	da_data[0] = ((int)(-da_scale[0] * (da_temp[0] - da_mid[0])));
	SpidRegs.SPITXBUF = 0x1000;
	SpidRegs.SPITXBUF = (int)((da_data[0]<<4)&0xFF00);
	SpidRegs.SPITXBUF = (int)((da_data[0]<<12)&0xFF00);
	delaycc(0.9e-6);

	da_temp[1] = da_type[1] ? (int)(*da[1]) : *(float *)(da[1]);
	da_data[1] = ((int)(-da_scale[1] * (da_temp[1] - da_mid[1])));
	SpidRegs.SPITXBUF = 0x1200;
	SpidRegs.SPITXBUF = (int)((da_data[1]<<4)&0xFF00);
	SpidRegs.SPITXBUF = (int)((da_data[1]<<12)&0xFF00);
	delaycc(0.9e-6);

	da_temp[2] = da_type[2] ? (int)(*da[2]) : *(float *)(da[2]);
 	da_data[2] = ((int)(-da_scale[2] * (da_temp[2] - da_mid[2])));
	SpidRegs.SPITXBUF = 0x1400;
	SpidRegs.SPITXBUF = (int)((da_data[2]<<4)&0xFF00);
	SpidRegs.SPITXBUF = (int)((da_data[2]<<12)&0xFF00);
	delaycc(0.9e-6);

	da_temp[3] = da_type[3] ? (int)(*da[3]) : *(float *)(da[3]);
	da_data[3] = ((int)(-da_scale[3] * (da_temp[3] - da_mid[3])));
	SpidRegs.SPITXBUF = 0x1600;
	SpidRegs.SPITXBUF = (int)((da_data[3]<<4)&0xFF00);
	SpidRegs.SPITXBUF = (int)((da_data[3]<<12)&0xFF00);
}
