#include "DSP2834x_Device.h"
#include "DSP2834x_Examples.h"
#include "variable.h"

void InitEQep(void)
{
    InitEQepGpio();

    EQep1Regs.QDECCTL.bit.QSRC = 00;        // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 0x2;
    EQep1Regs.QEPCTL.bit.PCRM = 00;         // QPOSCNT reset source: index event
    EQep1Regs.QEPCTL.bit.IEI = 0x2;         // Start counting on rising edge of index event
    EQep1Regs.QPOSINIT = 0x0;               // Z-pulse reset count
    EQep1Regs.QPOSMAX = (ENC_PPR<<2) - 1;   // Maximum position
    EQep1Regs.QEPCTL.bit.QPEN=0x1;          // QEP enable
}

void InitEQepGpio(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EQEP1A
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EQEP1B
	GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 1;   // Configure GPIO53 as EQEP1I

    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;   // Enable pull-up on GPIO53 (EQEP1I)

    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 1;   // Sync to SYSCLKOUT GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 1;   // Sync to SYSCLKOUT GPIO21 (EQEP1B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 1;   // Sync to SYSCLKOUT GPIO53 (EQEP1I)
	EDIS;
}

//===========================================================================
// End of file.
//===========================================================================
