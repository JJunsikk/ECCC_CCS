/******************************************************************************
    FAULT.C
******************************************************************************/
#include "DSP28x_Project.h"
#include "variable.h"
#include "fault.h"

Fault_info Fault_info_M;
Uint16 TZFault = 0, SWFault = 0;

interrupt void FaultInterrupt(void)
{
	Switch_DSP_Off();

	EPwm1Regs.CMPA.half.CMPA = 0;
	EPwm2Regs.CMPA.half.CMPA = 0;
	EPwm3Regs.CMPA.half.CMPA = 0;

    Flag.START_M = 0;
    INV_M.NextState = FAULT_STATE;

    if(SWFault == 0)    TZFault = 1;

	Fault_info_M.Ia_fault = INV_M.Ia;
	Fault_info_M.Ib_fault = INV_M.Ib;
	Fault_info_M.Ic_fault = INV_M.Ic;
    Fault_info_M.Idse_fault = INV_M.Idsr;
    Fault_info_M.Iqse_fault = INV_M.Iqsr;
	Fault_info_M.Vdc_fault = INV_M.Vdc;
	Fault_info_M.Wrpm_fault = INV_M.Wrpm;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2; // EPWM_TZ
}

void SWFaultFunc(void)
{
    SWFault = 1;
    EALLOW;
    EPwm1Regs.TZFRC.bit.CBC = 1;
    EDIS;
}

void InitFault(void)
{
    TZFault = 0;
    SWFault = 0;

	Fault_info_M.Ia_fault = 0.;
	Fault_info_M.Ib_fault = 0.;
	Fault_info_M.Ic_fault = 0.;
    Fault_info_M.Idse_fault = 0.;
    Fault_info_M.Iqse_fault = 0.;
	Fault_info_M.Vdc_fault = 0.;
	Fault_info_M.Wrpm_fault = 0.;
}

void ClearFault(void)
{
    Flag.FAULT_CLEAR = 0;

    InitFault();

    INV_M.NextState = IDLE_STATE;

    EALLOW;
    EPwm1Regs.TZCLR.bit.CBC = 1;
    EPwm2Regs.TZCLR.bit.CBC = 1;
    EPwm3Regs.TZCLR.bit.CBC = 1;

    EPwm1Regs.TZCLR.bit.INT = 1;
    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2; // EPWM_TZ
}
