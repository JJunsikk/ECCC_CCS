#include "DSP28x_Project.h"
#include "variable.h"


void ConfigureEPWM(volatile struct EPWM_REGS *EPwmXRegs, Uint16 maxCount_ePWM_temp, Uint16 Initial_CMP, float Deadband, Uint16 Counter_mode);

void InitEPwm(void)
{
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;                          // 고유 주파수를 가지는 PWM이 켜지면 모듈 내부의 기본 클럭 중지시킴
	EDIS;

	maxCount_samp = (Uint16)(0.5*PWM_CLK/Fsamp);                    // 1주기동안 카운트할 수 있는 양의 절반만 카운트. 1주기 이내에 삼각파를 만들기 위함.    // Fsamp가 20e3일 때, 7500 count 발생
	Tsamp = 2. * (float)maxCount_samp * PWM_CLK_PRD;                // 1주기 샘플링타임은 카운트의 두 배로.
    maxCount_SC = (Uint16)(0.5*(PWM_CLK/16.)/(3.*Fsc));             //
    Tsc = 2. * (float)(maxCount_SC) * (PWM_CLK_PRD * 16.) * 3.;     //

	InitEPwmGpio();

	EALLOW;
	ConfigureEPWM(&EPwm1Regs, maxCount_samp, 0, Tdead, TB_COUNT_UPDOWN);    // PWM 구성 함수        // 샘플링카운트 크기 조절하여 반송파 주파수 조절 가능. 16kHz 목표
	ConfigureEPWM(&EPwm2Regs, maxCount_samp, 0, Tdead, TB_COUNT_UPDOWN);
	ConfigureEPWM(&EPwm3Regs, maxCount_samp, 0, Tdead, TB_COUNT_UPDOWN);

    ConfigureEPWM(&EPwm9Regs, maxCount_SC, 0, 0, TB_COUNT_UPDOWN);

	//Interrupt (peak, algorithm)
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	EPwm1Regs.ETSEL.bit.INTEN = 1;                                  // Interrupt Enable
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;                        // 삼각파 최고점에서 인터럽트 발생
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;                             // 삼각파가 최고점에 도달하면 이벤트가 발생함. 이벤트 몇 개가 쌓여야 피크 인터럽트를 내보낼지 결정.
                                                                    // EPwm2, EPwm3은 EPwm1과 같은 삼각파를 생성하므로, period 최대값에 대한 인터럽트 발생 명령을 내릴 필요 없음.

    // Interrupt for speed control
    EPwm9Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;   //TBCLK = EPWMCLK/(HSPCLKDIV * CLKDIV)
    EPwm9Regs.TBCTL.bit.CLKDIV = TB_DIV4;
    EPwm9Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm9Regs.ETSEL.bit.INTEN = 1;                                  // Interrupt Enable
    EPwm9Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;
    EPwm9Regs.ETPS.bit.INTPRD = ET_3RD;                             // 최고점 이벤트가 3개째 발생하면 인터럽트 동작

	//Trip zone interrupt
	EPwm1Regs.TZEINT.bit.CBC = 1;

	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

    Switch_DSP_Off();
}


void InitEPwmGpio(void)
{
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
}

void ConfigureEPWM(volatile struct EPWM_REGS *EPwmXRegs, Uint16 maxCount_ePWM_temp, Uint16 Initial_CMP, float Deadtime, Uint16 Counter_mode)
{
	//TZ setting                                                    // (*EPwmXRegs).TZCLR.bit.CBC     // 구조체_포인터 -> 멤버
	EPwmXRegs->TZCLR.bit.CBC = 1;                                   // 발생해 있는 트립 클리어 동작. 매 실행마다 클리어
	EPwmXRegs->TZCLR.bit.INT = 1;                                   // 발생해 있는 전역 인터럽트 클리어 동작

	//EPwmXRegs->TZSEL.bit.CBC1 = 1; // nTZ1 is trip source for EPwmX     // 트립존1 클리어
	//EPwmXRegs->TZSEL.bit.CBC2 = 1; // nTZ2 is trip source for EPwmX     // 트립존2 클리어
	EPwmXRegs->TZCTL.bit.TZA = TZ_FORCE_LO;                         // 트립이 발생하면 PWM 신호를 강제로 LOW로 고정
	EPwmXRegs->TZCTL.bit.TZB = TZ_FORCE_LO;                         // 상동.

	// Setup Counter
	EPwmXRegs->TBPRD = maxCount_ePWM_temp;                          // 카운트 양
	EPwmXRegs->CMPA.half.CMPA = Initial_CMP;                        // 카운트 초기값
    EPwmXRegs->CMPB = Initial_CMP;
	EPwmXRegs->TBPHS.all = 0;
	EPwmXRegs->TBCTR = 0x0000;                                      // 카운터 값 초기화

	// Setup counter mode
	EPwmXRegs->TBCTL.bit.CTRMODE = Counter_mode;                    // up-down count mode
	EPwmXRegs->TBCTL.bit.PRDLD = TB_SHADOW;                         // use shadow register

	// Sync
	EPwmXRegs->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwmXRegs->TBPHS.half.TBPHS = 0;                                // 동기화 명령이 들어왔을 때, 카운트를 출발시킬 위치
	EPwmXRegs->TBCTL.bit.PHSDIR = TB_UP;                            // 카운트 방향
	EPwmXRegs->TBCTL.bit.PHSEN = TB_ENABLE;                         // slave mode

	// Setup counter clock
	EPwmXRegs->TBCTL.bit.HSPCLKDIV = TB_DIV1;  	//TBCLK = EPWMCLK/(HSPCLKDIV * CLKDIV)
	EPwmXRegs->TBCTL.bit.CLKDIV = TB_DIV1;

	//Setup shadowing
	EPwmXRegs->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwmXRegs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;

	//Set actions
	EPwmXRegs->AQCTLA.bit.CAD = AQ_CLEAR;
	EPwmXRegs->AQCTLA.bit.CAU = AQ_CLEAR;
	EPwmXRegs->AQCTLB.bit.CAD = AQ_CLEAR;
	EPwmXRegs->AQCTLB.bit.CAU = AQ_CLEAR;

	EPwmXRegs->AQSFRC.bit.OTSFA = 1;
	EPwmXRegs->AQSFRC.bit.OTSFB = 1;

	EPwmXRegs->AQSFRC.bit.ACTSFA = AQ_CLEAR;
	EPwmXRegs->AQSFRC.bit.ACTSFB = AQ_CLEAR;

	//Dead time
	EPwmXRegs->DBCTL.bit.OUT_MODE = DB_DISABLE;
	EPwmXRegs->DBCTL.bit.POLSEL = DB_ACTV_HIC;
	EPwmXRegs->DBCTL.bit.IN_MODE = DBA_ALL;
	EPwmXRegs->DBRED = (Uint16)(Deadtime*PWM_CLK);
	EPwmXRegs->DBFED = (Uint16)(Deadtime*PWM_CLK);
}

void InitEPwm1Gpio(void)
{
	EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
	EDIS;
}

void InitEPwm2Gpio(void)
{
	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
	EDIS;
}

void InitEPwm3Gpio(void)
{
	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
	EDIS;
}

void InitEPwm4Gpio(void)
{
	EALLOW;
	GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6
	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO7
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B
	EDIS;
}

void InitEPwm5Gpio(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up on GPIO8
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable pull-up on GPIO9
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B
    EDIS;
}

void InitEPwm6Gpio(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // Disable pull-up on GPIO10
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // Disable pull-up on GPIO11
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B
    EDIS;
}
//===========================================================================
// End of file.
//===========================================================================
