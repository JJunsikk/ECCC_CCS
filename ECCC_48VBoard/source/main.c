#include "DSP28x_Project.h"
#include "easy28x_gen2_bitfield_v9.4.h"
#include "variable.h"
#include "fault.h"

Uint32 loopCnt = 0;

void main(void)
{	
	InitSysCtrl();

	DINT;
	InitPieCtrl();

	IER = 0x0000;
	IFR = 0x0000;

	InitPieVectTable();

	EALLOW;
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

	// Order of priority
	PieVectTable.EPWM1_TZINT = &FaultInterrupt;         //(2,1)
	PieVectTable.EPWM1_INT = &OffsetInterrupt;          //(3,1)
    PieVectTable.EPWM9_INT = &FscInterrupt;             //(11,1)

	IER = M_INT2|M_INT3|M_INT9|M_INT11; // TZ | EPWM1_INT | SCI | EPWM9_INT
	EDIS;

	EINT;

	InitCpuTimers();
	ConfigCpuTimer(&CpuTimer0, 300., SYS_CLK_PRD*1.e6);
	CpuTimer0Regs.TCR.all = 0x4000;

	InitGpio();
	InitEPwm();
	XintfInit();

	InitAdc();
	InitSerialDac();
	InitEQep();
	easyDSP_SCI_Init();

	//////////////////// Init functions ////////////////////
    InitFault();

	InitFlag();

	InitParameter(&INV_M);

	InitUserDefinedVars(&INV_M);

	Init_Controller(&INV_M);

    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    if(INV_M.InitControlMode == SPDCONTL_MODE)
        PieCtrlRegs.PIEIER11.bit.INTx1 = 1;

	while(1)
	{
		loopCnt++;
	}
}
//===========================================================================
// No more.
//===========================================================================
