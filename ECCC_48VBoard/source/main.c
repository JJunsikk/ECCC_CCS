#include "DSP28x_Project.h"
#include "easy28x_gen2_bitfield_v9.4.h"
#include "variable.h"
#include "fault.h"

Uint32 loopCnt = 0;

void main(void)         // 엔코더 펄스를 직접 변경해가면서 PWM 발생되는지 테스트?
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
	PieVectTable.EPWM1_TZINT = &FaultInterrupt;         //(2,1)         // 실행시키고 싶은 인터럽트의 주소값을 벡터 테이블에 등록해둔다.
	PieVectTable.EPWM1_INT = &OffsetInterrupt;          //(3,1)         // 숫자가 작을수록 우선순위가 높다.
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
    InitFault();                                                        // HW, SW Fault 상태 초기화

	InitFlag();

	InitParameter(&INV_M);                                              // 직접 설정하지 않는 변수 초기화

	InitUserDefinedVars(&INV_M);                                        // 사용자가 직접 설정하는 변수 초기화

	Init_Controller(&INV_M);                                            // 제어기 변수 초기화

    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    if(INV_M.InitControlMode == SPDCONTL_MODE)                          // 초기 모드가 속도제어 모드일 경우, 속도제어 인터럽트가 포함된 그룹을 ENABLE한다.
        PieCtrlRegs.PIEIER11.bit.INTx1 = 1;

	while(1)
	{
		loopCnt++;
	}
}
//===========================================================================
// No more.
//===========================================================================
