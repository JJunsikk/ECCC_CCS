// TI File $Revision: /main/1 $
// Checkin $Date: February 1, 2008   09:59:35 $
//###########################################################################
//
// FILE:	DSP2834x_Gpio.c
//
// TITLE:	DSP2834x General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2834x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "DSP2834x_Examples.h"   // DSP2834x Examples Include File

//---------------------------------------------------------------------------
// InitGpio: 
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known (default) state.
//
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example. 

void InitGpio(void)
{
	EALLOW;

	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;	//GPIO12 - nTZ1
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;
	GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;

	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;	//GPIO13 - nTZ2
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;
	GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;
	GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;

	GpioCtrlRegs.GPBMUX1.bit.GPIO46 = 0;	//GPIO46 - ADC0SOC
	GpioCtrlRegs.GPBDIR.bit.GPIO46 = 1;
	GpioCtrlRegs.GPBPUD.bit.GPIO46 = 0;

	GpioCtrlRegs.GPBMUX1.bit.GPIO47 = 0;	//GPIO47 - ADC0BZ
	GpioCtrlRegs.GPBDIR.bit.GPIO47 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO47 = 0;

    GpioCtrlRegs.GPCMUX2.bit.GPIO81 = 0;    //GPIO81 - ADC1SOC
    GpioCtrlRegs.GPCDIR.bit.GPIO81 = 1;
    GpioCtrlRegs.GPCPUD.bit.GPIO81 = 0;

    GpioCtrlRegs.GPCMUX2.bit.GPIO82 = 0;    //GPIO82 - ADC1BZ
    GpioCtrlRegs.GPCDIR.bit.GPIO82 = 0;

    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 3;    //GPIO48 - SPISIMOD
    GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 3;
    GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;

    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 3;    //GPIO50 - SPICLKD
    GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 3;
    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;

    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 3;    //GPIO51 - nSPISTED
    GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 3;
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;    //GPIO24 - PWM123En
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;    //GPIO25 - PWM456En
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;
	EDIS;
}

//===========================================================================
// End of file.
//===========================================================================
