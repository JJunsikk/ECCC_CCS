#include "fault.h"
#include "variable.h"
#include "comp.h"

Uint16 FscCnt = 0;
long FscIntTimerStart = 0;
float TimeFscInt = 0.;

interrupt void FscInterrupt(void) {
    FscIntTimerStart = ReadCpuTimer0Counter();

    FscCnt++;

    SlopeGenerator(&INV_M.WrpmRef, INV_M.WrpmRefSet, INV_M.DelWrpmRef);
    INV_M.WrmRef = INV_M.WrpmRef * RPM2RM;
    INV_M.ErrWrm = INV_M.WrmRef - INV_M.WrmLPFilt;
    INV_M.TeRefInteg += Tsc * INV_M.KiSc * (INV_M.ErrWrm - INV_M.KaSc * INV_M.TeRefAW);
    INV_M.TeRefUnsat = -INV_M.KpSc * INV_M.Wrm + INV_M.TeRefInteg;
    INV_M.TeRef = LIMIT(INV_M.TeRefUnsat, INV_M.TeRefMin, INV_M.TeRefMax);
    INV_M.TeRefAW = INV_M.TeRefUnsat - INV_M.TeRef;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP11;
    EPwm9Regs.ETCLR.bit.INT = 1;

    TimeFscInt = (float)(FscIntTimerStart - ReadCpuTimer0Counter())*SYS_CLK_PRD;
}
