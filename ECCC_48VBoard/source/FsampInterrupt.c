#include "DSP28x_Project.h"
#include "filter.h"
#include "fault.h"
#include "variable.h"
#include "comp.h"

Uint16 FsampCnt = 0;
long FsampIntTimerStart = 0;
float TimeFsampInt = 0.;

interrupt void FsampPeakInterrupt(void) {
    FsampIntTimerStart = ReadCpuTimer0Counter();

    FsampCnt++;

    if(Flag.RESET == 1) {
        Flag.START_M = 0;
        Flag.RESET = 0;

        if(Flag.FAULT_CLEAR == 1)        ClearFault();

        Init_Controller(&INV_M);
    }

    ////////////////////////////// ADC Process //////////////////////////////
    ADC0_Process();
    INV_M.Ia = ScaleAD0[4]* ((float) AD0CH[4] - OffsetAD0[4]);
    INV_M.Ib = ScaleAD0[5]* ((float) AD0CH[5] - OffsetAD0[5]);
    INV_M.Ic = ScaleAD0[3]* ((float) AD0CH[3] - OffsetAD0[3]);
    INV_M.Vdc = ScaleAD0[1]* (float) AD0CH[1];

    INV_M.Thetarm = BOUND_PI(INV_M.Enc_scale * (float)EQep1Regs.QPOSCNT - INV_M.ThetarmOffset);

    if(INV_M.Vdc < 1.)      INV_M.InvVdc = 1.;
    else                    INV_M.InvVdc = 1. / INV_M.Vdc;

    if((ABS(INV_M.Ia) >= 60.) || (ABS(INV_M.Ib) >= 60.) || (ABS(INV_M.Ic) >= 60.) || (ABS(INV_M.Vdc) >= 50.) || ABS(INV_M.Wrpm) >= 3500.)
        SWFaultFunc();

    INV_M.Idss = INV_M.Ia;
    INV_M.Iqss = (INV_M.Ib - INV_M.Ic) * INV_SQRT3;

    ////////////////////////////// State machine //////////////////////////////
    INV_M.PrevState = INV_M.CurrState;
    INV_M.CurrState = INV_M.NextState;

    switch (INV_M.CurrState) {
    case IDLE_STATE:

        if(INV_M.PrevState != IDLE_STATE) {
            Switch_DSP_Off();
            Init_Controller(&INV_M);
        }

        if(Flag.START_M == 1) {
            BootstrapCharge();
            if(INV_M.BootStrapEnd == 1) INV_M.NextState = ALIGN_STATE;
        }
        else {
            Switch_DSP_Off();
            INV_M.NextState = IDLE_STATE;
        }

        break;

    case ALIGN_STATE:
        Align(&INV_M);
        CurrentControl(&INV_M);
        VoltageModulation(&INV_M);
        PwmUpdate(INV_M.DutyA, INV_M.DutyB, INV_M.DutyC);

        if(Flag.START_M == 0)           INV_M.NextState = IDLE_STATE;
        else if(INV_M.AlignEnd == 1)    INV_M.NextState = RUN_STATE;
        else                            INV_M.NextState = ALIGN_STATE;

        Switch_DSP_On();

        break;

    case RUN_STATE:
        SpeedObserver(&INV_M);
        CurrentRefGenerator(&INV_M);
        CurrentControl(&INV_M);
        VoltageModulation(&INV_M);
        PwmUpdate(INV_M.DutyA, INV_M.DutyB, INV_M.DutyC);

        if(Flag.START_M == 0)               INV_M.NextState = IDLE_STATE;
        else                                INV_M.NextState = RUN_STATE;

        Switch_DSP_On();
        break;

    default:
        INV_M.NextState = FAULT_STATE;
        break;
    }

    if((INV_M.NextState != FAULT_STATE) && (INV_M.Vdc < 1.))  INV_M.NextState = IDLE_STATE;

    SerialDacOut();

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    EPwm1Regs.ETCLR.bit.INT = 1;

    TimeFsampInt = (float)(FsampIntTimerStart - ReadCpuTimer0Counter())*SYS_CLK_PRD;
}
