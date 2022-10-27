// TI File $Revision: /main/5 $
// Checkin $Date: August 28, 2008   16:54:21 $
//###########################################################################
//
// FILE:   DSP2834x_Xintf.c
//
// TITLE:   DSP2834x Device External Interface Init & Support Functions.
//
// DESCRIPTION:
//
//          Example initialization function for the external interface (XINTF).
//          This example configures the XINTF to its default state.  For an
//          example of how this function being used refer to the
//          examples/run_from_xintf project.
//
//###########################################################################
// $TI Release: DSP2834x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "DSP2834x_Examples.h"   // DSP2834x Examples Include File

//---------------------------------------------------------------------------
// InitXINTF:
//---------------------------------------------------------------------------
// This function initializes the External Interface the default reset state.
//
// Do not modify the timings of the XINTF while running from the XINTF.  Doing
// so can yield unpredictable results

void XintfInit(void)
{
	InitXintf16Gpio();

    EALLOW;
    XintfRegs.XINTCNF2.bit.WRBUFF = 3;

	XintfRegs.XTIMING7.bit.XWRLEAD = 1;
    XintfRegs.XTIMING7.bit.XWRACTIVE = 2;
    XintfRegs.XTIMING7.bit.XWRTRAIL = 1;

    XintfRegs.XTIMING7.bit.XRDLEAD = 1;
    XintfRegs.XTIMING7.bit.XRDACTIVE = 3;
    XintfRegs.XTIMING7.bit.XRDTRAIL = 1;

    XintfRegs.XTIMING7.bit.X2TIMING = 0;

    XintfRegs.XTIMING7.bit.USEREADY = 1;
    XintfRegs.XTIMING7.bit.READYMODE = 0;  // sample asynchronous

    XintfRegs.XTIMING7.bit.XSIZE = 1;

    XintfRegs.XBANK.bit.BANK = 7;
    XintfRegs.XBANK.bit.BCYC = 0;
    EDIS;

    asm(" RPT #7 || NOP");

}

void InitXintf16Gpio()
{
     EALLOW;

     GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 2;  // XA1
     GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 2;  // XA2
     GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 2;  // XA3

     GpioCtrlRegs.GPCMUX1.bit.GPIO68 = 2;  // XD11
     GpioCtrlRegs.GPCMUX1.bit.GPIO69 = 2;  // XD10
     GpioCtrlRegs.GPCMUX1.bit.GPIO70 = 2;  // XD9
     GpioCtrlRegs.GPCMUX1.bit.GPIO71 = 2;  // XD8
     GpioCtrlRegs.GPCMUX1.bit.GPIO72 = 2;  // XD7
     GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 2;  // XD6
     GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 2;  // XD5
     GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 2;  // XD4
     GpioCtrlRegs.GPCMUX1.bit.GPIO76 = 2;  // XD3
     GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 2;  // XD2
     GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 2;  // XD1
     GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 2;  // XD0

     GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 2;  // XZCS7
     GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 2;  // XWE0
     EDIS;
}

//===========================================================================
// No more.
//===========================================================================
