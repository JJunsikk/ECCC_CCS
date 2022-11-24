#include "variable.h"
#include "comp.h"
#include "math.h"

FLAG_REG Flag;
INV INV_M;

// PWM
float Tsamp = 0., Tsc = 0.;
Uint16 maxCount_samp = 0, maxCount_SC = 0;

int32 CmprTemp = 0;

//delaycc function
unsigned short cnt_delay;
void delaycc(float time) {
    cnt_delay=(unsigned short)(time*SYS_CLK);
    asm(" RPT @_cnt_delay   || NOP ");
    asm(" NOP       ");
}

void SlopeGenerator(float *Var, float Cmd, float DelPerTs) {
    if(*Var < (Cmd - DelPerTs))
        *(float *)Var += DelPerTs;
    else if(*Var > (Cmd + DelPerTs))
        *(float *)Var -= DelPerTs;
    else
        *(float *)Var = Cmd;
}

void InitParameter(INV *tmpINV) {
    //////////////////// Target motor ////////////////////
    tmpINV->P = 8.;
    tmpINV->Rs = 0.95;
    tmpINV->Ld = 3.1e-3;
    tmpINV->Lq = 3.1e-3;
    tmpINV->Lamf = 0.036;
    tmpINV->Jm = 1e-2;
    tmpINV->Bm = 1e-6;
    tmpINV->Irated = 50.;
}

void InitUserDefinedVars(INV *tmpINV) {
    //////////////////// Target motor ////////////////////
    /* Variables for align */
    tmpINV->AlignCntMax = (Uint32)(2.* Fsamp);
    tmpINV->WrRefSetAlign = (PI2 * 1.);
    tmpINV->DelWrRefAlign = (PI2 * 10.) * Tsamp;
    tmpINV->IdsrRefSetAlign = 1.;
    tmpINV->DelIdsrAlign = 200. * Tsamp;

    /* Variables for constant I mode */
    tmpINV->IdsrRefSetIbyF = 10.;
    tmpINV->DelIdsrRefIbyF = 100. * Tsamp;
    tmpINV->WrRefSetIbyF = (PI2 * 20.);
    tmpINV->DelWrRefIbyF = (PI2 * 20.) * Tsamp;

    /* Variables for speed controller */
    tmpINV->WcSc = PI2 * 20.;
    tmpINV->TeRefMin = -1.5 * tmpINV->PP * tmpINV->Lamf * tmpINV->Irated;
    tmpINV->TeRefMax = 1.5 * tmpINV->PP * tmpINV->Lamf * tmpINV->Irated;
    tmpINV->DelWrpmRef = PI2 * 100. * Tsc;

    /* Variables for speed observer */
    tmpINV->WcSO1 = PI2 * 100.;
    tmpINV->WcSO23 = 0.1 * tmpINV->WcSO1;
    tmpINV->ZetaSO = 10.;
    tmpINV->WcSpdLPF = tmpINV->WcSO1;

    /* Variables for Field weakening */
    tmpINV->VLimRatioFW = INV_SQRT3;
    tmpINV->KpFW = 0.01;
    tmpINV->KiFW = 0.01;

    /* Variables for Current controller */
    tmpINV->WcCc = PI2 * 300.;
    tmpINV->VLimRatioCC = 1.05 * INV_SQRT3;

    /* Variables for initial control mode */
    tmpINV->InitControlMode = SPDCONTL_MODE;
}

void Init_Controller(INV *tmpINV) {
    tmpINV->INVOnOffState = INV_OFF_STATE;

    /* Variables for state machine */
    tmpINV->PrevState = IDLE_STATE;
    tmpINV->CurrState = IDLE_STATE;
    tmpINV->NextState = IDLE_STATE;

    /* Variables for motor parameter */
    tmpINV->PP = 0.5 * tmpINV->P;
    tmpINV->InvLamf = 1. / tmpINV->Lamf;
    tmpINV->InvLd = 1. / tmpINV->Ld;
    tmpINV->InvLq = 1. / tmpINV->Lq;
    tmpINV->InvPP = 1. / tmpINV->PP;
    tmpINV->InvJm = 1. / tmpINV->Jm;
    tmpINV->Kt = 1.5 * tmpINV->PP * tmpINV->Lamf;
    tmpINV->InvKt = 1. / tmpINV->Kt;

    /* Variables for sensor */
    tmpINV->Vdc = 0., tmpINV->InvVdc = 0.;
    tmpINV->Ia = 0., tmpINV->Ib = 0., tmpINV->Ic = 0.;
    tmpINV->Enc_scale = -PI2 / (float)EQep1Regs.QPOSMAX; // +- check!! 220801 by CHG.
    tmpINV->Thetarm = 0.;

    /* Variables for bootstrap initial charge */
    tmpINV->BootStrapEnd = 0;
    tmpINV->BootStrapStepCnt = 0;
    tmpINV->BootStrapChargeEpwmCnt = (Uint16)(0.2*maxCount_samp); // enough time check!! 220801 by CHG.

    /* Variables for align */
    tmpINV->AlignEnd = 0;
    tmpINV->AlignStep = 0, tmpINV->AlignCnt = 0;
    tmpINV->IdsrRefAlign = 0.;
    tmpINV->WrRefAlign = 0., tmpINV->ThetarAlign = 0.;
    tmpINV->ThetarmOffsetTemp = 0., tmpINV->ThetarmOffset = 0.;
    tmpINV->INV_AlignCntPlus1 = 0.;

    /* Variables for Constant I mode */
    tmpINV->IdsrRefIbyF = 0.;
    tmpINV->WrRefIbyF = 0.;
    tmpINV->ThetarIbyF = 0., tmpINV->ThetarIbyFSqr = 0., tmpINV->ThetarCompIbyF = 0., tmpINV->ThetarCompIbyFSqr = 0.;

    /* Variables for speed observer */
    tmpINV->K1 = tmpINV->WcSO1 + 2. * tmpINV->ZetaSO * tmpINV->WcSO23 - tmpINV->Bm * tmpINV->InvJm;
    tmpINV->K2 = tmpINV->WcSO23 * tmpINV->WcSO23 + 2. * tmpINV->ZetaSO * tmpINV->WcSO23 * tmpINV->WcSO1 - tmpINV->K1 * tmpINV->Bm * tmpINV->InvJm;
    tmpINV->K3 = tmpINV->WcSO1 * tmpINV->WcSO23 * tmpINV->WcSO23;

    tmpINV->ErrThetarm = 0., tmpINV->AccEstInteg = 0., tmpINV->AccFF = 0.;
    tmpINV->Wrpm = 0., tmpINV->Wrm = 0., tmpINV->Wr = 0.;
    tmpINV->InvWr = 0.;
    tmpINV->ThetarmEst = 0., tmpINV->ThetarEst = 0.;
    tmpINV->WrmLPFilt = 0.;
    initiateIIR1(&tmpINV->WrmLPF, K_LPF, tmpINV->WcSpdLPF, Tsamp);

    /* Variables for speed controller */
    tmpINV->KpSc = tmpINV->Jm * tmpINV->WcSc;
    tmpINV->KiSc = 0.2 * tmpINV->KpSc * tmpINV->WcSc;
    tmpINV->KaSc = 1. / tmpINV->KpSc;
    tmpINV->WrpmRef = 0., tmpINV->WrpmRefSet = 0., tmpINV->WrmRef = 0.;
    tmpINV->ErrWrm = 0., tmpINV->TeRefInteg = 0., tmpINV->TeRefUnsat = 0., tmpINV->TeRef = 0., tmpINV->TeRefAW = 0.;

    /* Variables for Field weakening */
    tmpINV->KaFW = 1. / tmpINV->KpFW;
    tmpINV->VdqsrRefMax = 0.;
    tmpINV->VmagErr = 0., tmpINV->DelIdsrRefFWInteg = 0., tmpINV->DelIdsrRefFWUnsat = 0., tmpINV->DelIdsrRefFW = 0., tmpINV->DelIdsrRefFWAW = 0.;
    tmpINV->IdsrRefMax = 0., tmpINV->IqsrRefMax = 0.;

    /* Variables for Current controller */
    tmpINV->Thetar = 0., tmpINV->ThetarSqr = 0.;
    tmpINV->ThetarComp = 0., tmpINV->ThetarCompSqr = 0.;
    tmpINV->CosThetar = 1., tmpINV->SinThetar = 0.;
    tmpINV->CosThetarComp = 1., tmpINV->SinThetarComp = 0.;

    tmpINV->KpdCc = tmpINV->Ld * tmpINV->WcCc;
    tmpINV->KidCc = tmpINV->Rs * tmpINV->WcCc;
    tmpINV->KadCc = 1. / tmpINV->KpdCc ;
    tmpINV->KpqCc = tmpINV->Lq * tmpINV->WcCc;
    tmpINV->KiqCc = tmpINV->Rs * tmpINV->WcCc;
    tmpINV->KaqCc = 1. / tmpINV->KpqCc;

    tmpINV->IdsrRefSet = 0., tmpINV->IqsrRefSet = 0.;
    tmpINV->IdsrRef = 0., tmpINV->IqsrRef = 0.;
    tmpINV->Idss = 0., tmpINV->Iqss = 0.;
    tmpINV->Idsr = 0., tmpINV->Iqsr = 0.;
    tmpINV->ErrIdsr = 0., tmpINV->VdsrRefInteg = 0., tmpINV->VdsrRefFF = 0., tmpINV->VdsrRefAw = 0., tmpINV->VdsrRef = 0.;
    tmpINV->ErrIqsr = 0., tmpINV->VqsrRefInteg = 0., tmpINV->VqsrRefFF = 0., tmpINV->VqsrRefAw = 0., tmpINV->VqsrRef = 0.;
    tmpINV->VdqsrOutMag = 0.;

    /* Variables for voltage modulation */
    tmpINV->VdssRef = 0., tmpINV->VqssRef = 0.;
    tmpINV->VasRef = 0., tmpINV->VbsRef = 0., tmpINV->VcsRef = 0.;
    tmpINV->VabcsRefMax = 0., tmpINV->VabcsRefMin = 0., tmpINV->VOffset = 0.;
    tmpINV->VanRef = 0., tmpINV->VbnRef = 0., tmpINV->VcnRef = 0.;
    tmpINV->DutyA = 0., tmpINV->DutyB = 0., tmpINV->DutyC = 0.;
    tmpINV->VanOut = 0., tmpINV->VbnOut = 0., tmpINV->VcnOut = 0.;
    tmpINV->VdssOut = 0., tmpINV->VqssOut = 0.;
    tmpINV->VdsrOut = 0., tmpINV->VqsrOut = 0.;
}

void InitFlag(void) {
    Flag.START_M = 0;
    Flag.RESET = 0;
    Flag.FAULT_CLEAR = 0;
}

void UpdateGain(INV *tmpINV) {
}

void BootstrapCharge(void) {
    switch(INV_M.BootStrapStepCnt) {
    case 0:
        Switch_DSP_Off();
        INV_M.BootStrapStepCnt++;
        break;

    case 1:
        INV_M.INVOnOffState = INV_ON_STATE;

        EPwm1Regs.CMPA.half.CMPA = 0;                           // 윗 상 PWM은 0으로 유지
        EPwm2Regs.CMPA.half.CMPA = 0;
        EPwm3Regs.CMPA.half.CMPA = 0;

        EPwm1Regs.CMPB = INV_M.BootStrapChargeEpwmCnt;
        EPwm2Regs.CMPB = INV_M.BootStrapChargeEpwmCnt;
        EPwm3Regs.CMPB = INV_M.BootStrapChargeEpwmCnt;

        EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
        EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;                    // 비교일치 시 LOW
        EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;                    // 비교일치 시 LOW
        EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;                    // 비교일치 시 LOW
        EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;                      // 비교일치 시 HIGH

        EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
        EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
        EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
        EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;

        EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
        EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
        EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
        EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;

        INV_M.BootStrapStepCnt++;

        PWM123_ENABLE;
        break;

    case 2:
    case 3:
        INV_M.BootStrapStepCnt++;
        break;

    case 4:
        INV_M.BootStrapStepCnt++;
        INV_M.BootStrapEnd = 1;
        Switch_DSP_Off();
        break;

    default:
        break;
    }
}

void Align(INV *tmpINV) {
    switch(tmpINV->AlignStep) {
    case 0:                                     // Align 동작 초기화 단계
        tmpINV->ThetarmOffset = 0.;
        tmpINV->IdsrRefAlign = 0.;
        tmpINV->WrRefAlign = 0.;

        tmpINV->AlignCnt = 0;
        EQep1Regs.QCLR.bit.IEL = 1;             // 인터럽트 플래그 삭제

        tmpINV->AlignStep++;
        break;

    case 1:                 // Var(제어 대상)       // Cmd(제어 목표)           // DelPerTs(변화량)
        SlopeGenerator(&tmpINV->IdsrRefAlign, tmpINV->IdsrRefSetAlign, tmpINV->DelIdsrAlign);       // tmpINV->IdsrRefSetAlign = 20.;
        if(tmpINV->IdsrRefAlign == tmpINV->IdsrRefSetAlign)   tmpINV->AlignStep++;                  // tmpINV->DelIdsrAlign = 200. * Tsamp;
        break;

    case 2:
        SlopeGenerator(&tmpINV->WrRefAlign, tmpINV->WrRefSetAlign, tmpINV->DelWrRefAlign);          // tmpINV->WrRefSetAlign = (PI2 * 5.);
                                                                                                    // tmpINV->DelWrRefAlign = (PI2 * 10.) * Tsamp;
        //if(EQep1Regs.QFLG.bit.IEL == 1) tmpINV->AlignStep++;                                      // z펄스가 발생하면 설정되는 레지스터. 즉, z펄스가 발생하여 초기 위치를 알았으면 다음 스텝으로 진행
        break;

    case 3:
        SlopeGenerator(&tmpINV->WrRefAlign, 0., tmpINV->DelWrRefAlign);
        if(tmpINV->WrRefAlign == 0.) {
            tmpINV->ThetarIbyF = 0.;
            tmpINV->AlignStep++;
        }
        break;

    case 4:
        tmpINV->AlignCnt++;
        if(tmpINV->AlignCnt == (tmpINV->AlignCntMax>>4)) {                                          // AlignCntMax : 2초에 해당하는 40,000    // 4bit shift했으므로, 2,500 count 후 다음 Align 스텝으로 넘어간다.
            tmpINV->AlignStep++;
            tmpINV->AlignCnt = 0;
        }
        break;

    case 5:
        tmpINV->INV_AlignCntPlus1 = 1. / ((float)tmpINV->AlignCnt + 1.);                            // 분모에 1을 더해 DIV0 방지.
        tmpINV->ThetarmOffsetTemp = (tmpINV->ThetarmOffsetTemp * (float)tmpINV->AlignCnt + tmpINV->Thetarm) * tmpINV->INV_AlignCntPlus1;    // Align 동작을 수행하는 동안, 읽어온 각도들의 평균을 계속 계산한다.
        tmpINV->AlignCnt++;

        if(tmpINV->AlignCnt == tmpINV->AlignCntMax) tmpINV->AlignStep++;                            // 40,000 count가 채워질 때까지 평균 계산 동작 수행

        break;

    default:
        tmpINV->ThetarmOffset = tmpINV->ThetarmOffsetTemp;                                          // 계산된 평균값 저장
        tmpINV->ThetarmOffsetTemp = 0.;

        tmpINV->IdsrRefAlign = 0.;
        tmpINV->WrRefAlign = 0.;
        tmpINV->AlignEnd = 1;

        tmpINV->AlignStep = 0;
        break;
    }

    tmpINV->IdsrRef = tmpINV->IdsrRefAlign;
    tmpINV->IqsrRef = 0.;

    tmpINV->Wr = tmpINV->WrRefAlign;
    if (ABS(tmpINV->Wr) < 1e-6)     tmpINV->InvWr = SIGN(tmpINV->Wr) * 1.e6;                        // SIGN(x) : (((x)<0)? -1. : 1. )
    else                            tmpINV->InvWr = 1. / tmpINV->Wr;

    tmpINV->ThetarIbyF = BOUND_PI(tmpINV->ThetarIbyF + Tsamp * tmpINV->Wr);
    tmpINV->ThetarIbyFSqr = tmpINV->ThetarIbyF * tmpINV->ThetarIbyF;
    tmpINV->CosThetar = COS(tmpINV->ThetarIbyFSqr);                                                 // COS(x) : (1.-(x2)*(f2-(x2)*(f4-(x2)*(f6-(x2)*(f8-(x2)*(f10-(x2)*(f12-(x2)*f14)))))))
    tmpINV->SinThetar = SIN(tmpINV->ThetarIbyF, tmpINV->ThetarIbyFSqr);                             // SIN(x, x2) : ((x)*(1.-(x2)*(f3-(x2)*(f5-(x2)*(f7-(x2)*(f9-(x2)*(f11-(x2)*(f13-(x2)*f15))))))))

    tmpINV->ThetarCompIbyF = BOUND_PI(tmpINV->ThetarIbyF + 1.5 * Tsamp * tmpINV->Wr);
    tmpINV->ThetarCompIbyFSqr = tmpINV->ThetarCompIbyF * tmpINV->ThetarCompIbyF;
    tmpINV->CosThetarComp = COS(tmpINV->ThetarCompIbyFSqr);
    tmpINV->SinThetarComp = SIN(tmpINV->ThetarCompIbyF, tmpINV->ThetarCompIbyFSqr);
}

void SpeedObserver(INV *tmpINV) {
    switch(tmpINV->InitControlMode) {

    case CONST_CUR_MODE:
        if(tmpINV->WrRefIbyF != tmpINV->WrRefSetIbyF)
            SlopeGenerator(&tmpINV->WrRefIbyF, tmpINV->WrRefSetIbyF, tmpINV->DelWrRefIbyF);

        tmpINV->Wr = tmpINV->WrRefIbyF;
        if (ABS(tmpINV->Wr) < 1e-6)     tmpINV->InvWr = SIGN(tmpINV->Wr) * 1.e6;
        else                            tmpINV->InvWr = 1. / tmpINV->Wr;

        tmpINV->ThetarIbyF = BOUND_PI(tmpINV->ThetarIbyF + Tsamp * tmpINV->Wr);
        tmpINV->ThetarIbyFSqr = tmpINV->ThetarIbyF * tmpINV->ThetarIbyF;
        tmpINV->CosThetar = COS(tmpINV->ThetarIbyFSqr);
        tmpINV->SinThetar = SIN(tmpINV->ThetarIbyF, tmpINV->ThetarIbyFSqr);

        tmpINV->ThetarCompIbyF = BOUND_PI(tmpINV->ThetarIbyF + 1.5 * Tsamp * tmpINV->Wr);
        tmpINV->ThetarCompIbyFSqr = tmpINV->ThetarCompIbyF * tmpINV->ThetarCompIbyF;
        tmpINV->CosThetarComp = COS(tmpINV->ThetarCompIbyFSqr);
        tmpINV->SinThetarComp = SIN(tmpINV->ThetarCompIbyF, tmpINV->ThetarCompIbyFSqr);

        break;

    case VECTCONTL_MODE:
    case SPDCONTL_MODE:
        tmpINV->ErrThetarm = BOUND_PI(tmpINV->Thetarm - tmpINV->ThetarmEst);

        tmpINV->AccEstInteg += Tsamp * tmpINV->K3 * tmpINV->ErrThetarm;
        tmpINV->AccFF = 0;

        tmpINV->Wrm += Tsamp * (tmpINV->K2 * tmpINV->ErrThetarm + tmpINV->AccEstInteg + tmpINV->AccFF - tmpINV->Bm * tmpINV->Wrm * tmpINV->InvJm);
        tmpINV->WrmLPFilt = IIR1Update(&tmpINV->WrmLPF, tmpINV->Wrm);
        tmpINV->Wr = tmpINV->WrmLPFilt * tmpINV->PP;
        tmpINV->Wrpm = tmpINV->WrmLPFilt * RM2RPM;

        if (ABS(tmpINV->Wr) < 1e-6)
            tmpINV->InvWr = SIGN(tmpINV->Wr) * 1.e6;
        else
            tmpINV->InvWr = 1. / tmpINV->Wr;

        tmpINV->ThetarmEst += Tsamp * (tmpINV->Wrm + tmpINV->K1 * tmpINV->ErrThetarm);
        tmpINV->ThetarmEst = BOUND_PI(tmpINV->ThetarmEst);
        tmpINV->ThetarEst = BOUND_PI(tmpINV->ThetarmEst * tmpINV->PP);

        tmpINV->Thetar = BOUND_PI(tmpINV->Thetarm * tmpINV->PP);
        tmpINV->ThetarSqr = tmpINV->Thetar * tmpINV->Thetar;
        tmpINV->ThetarComp = tmpINV->Thetar + 1.5 * tmpINV->Wr * Tsamp;
        tmpINV->ThetarCompSqr = tmpINV->ThetarComp * tmpINV->ThetarComp;

        tmpINV->CosThetar = COS(tmpINV->ThetarSqr);
        tmpINV->SinThetar = SIN(tmpINV->Thetar, tmpINV->ThetarSqr);
        tmpINV->CosThetarComp = COS(tmpINV->ThetarCompSqr);
        tmpINV->SinThetarComp = SIN(tmpINV->ThetarComp, tmpINV->ThetarCompSqr);

        break;
    }
}

void CurrentRefGenerator(INV *tmpINV) {
    switch(tmpINV->InitControlMode) {

    case CONST_CUR_MODE:
        if(tmpINV->IdsrRefIbyF != tmpINV->IdsrRefSetIbyF)
            SlopeGenerator(&tmpINV->IdsrRefIbyF, tmpINV->IdsrRefSetIbyF, tmpINV->DelIdsrRefIbyF);

        tmpINV->IdsrRef = tmpINV->IdsrRefIbyF;
        tmpINV->IqsrRef = 0.;

        break;

    case VECTCONTL_MODE:
        tmpINV->VmagErr = tmpINV->VLimRatioFW * tmpINV->Vdc - tmpINV->VdqsrOutMag;
        tmpINV->DelIdsrRefFWInteg += Tsamp * tmpINV->KiFW * (tmpINV->VmagErr - tmpINV->KaFW * tmpINV->DelIdsrRefFWAW);
        tmpINV->DelIdsrRefFWUnsat = tmpINV->KpFW * tmpINV->VmagErr + tmpINV->DelIdsrRefFWInteg;
        tmpINV->DelIdsrRefFW = LIMIT(tmpINV->DelIdsrRefFWUnsat, -tmpINV->Irated, 0.);
        tmpINV->DelIdsrRefFWAW = tmpINV->DelIdsrRefFWUnsat - tmpINV->DelIdsrRefFW;
        tmpINV->IdsrRef = LIMIT(tmpINV->IdsrRefSet + tmpINV->DelIdsrRefFW, -tmpINV->Irated, tmpINV->Irated);
        tmpINV->IqsrRefMax = sqrt(tmpINV->Irated * tmpINV->Irated - tmpINV->IdsrRef * tmpINV->IdsrRef);

        tmpINV->IqsrRef = LIMIT(tmpINV->IqsrRefSet, -tmpINV->IqsrRefMax, tmpINV->IqsrRefMax);

        break;

    case SPDCONTL_MODE:
        tmpINV->VmagErr = tmpINV->VLimRatioFW * tmpINV->Vdc - tmpINV->VdqsrOutMag;
        tmpINV->DelIdsrRefFWInteg += Tsamp * tmpINV->KiFW * (tmpINV->VmagErr - tmpINV->KaFW * tmpINV->DelIdsrRefFWAW);
        tmpINV->DelIdsrRefFWUnsat = tmpINV->KpFW * tmpINV->VmagErr + tmpINV->DelIdsrRefFWInteg;
        tmpINV->DelIdsrRefFW = LIMIT(tmpINV->DelIdsrRefFWUnsat, -tmpINV->Irated, 0.);
        tmpINV->DelIdsrRefFWAW = tmpINV->DelIdsrRefFWUnsat - tmpINV->DelIdsrRefFW;

        tmpINV->IqsrRef = tmpINV->InvKt * tmpINV->TeRef;
        tmpINV->IdsrRefMax = sqrt(tmpINV->Irated * tmpINV->Irated - tmpINV->IqsrRef * tmpINV->IqsrRef);
        tmpINV->IdsrRef = LIMIT((tmpINV->IdsrRefSet + tmpINV->DelIdsrRefFW), -tmpINV->IdsrRefMax, tmpINV->IdsrRefMax);

        break;
    }
}

void CurrentControl(INV *tmpINV) {
    tmpINV->VdqsrRefMax = tmpINV->VLimRatioCC * tmpINV->Vdc;

    tmpINV->VdsrRefAw = tmpINV->VdsrRef - tmpINV->VdsrOut;
    tmpINV->VqsrRefAw = tmpINV->VqsrRef - tmpINV->VqsrOut;

    tmpINV->Idsr = tmpINV->Idss * tmpINV->CosThetar + tmpINV->Iqss * tmpINV->SinThetar;
    tmpINV->Iqsr = -tmpINV->Idss * tmpINV->SinThetar + tmpINV->Iqss * tmpINV->CosThetar;

    tmpINV->ErrIdsr = tmpINV->IdsrRef - tmpINV->Idsr;
    tmpINV->ErrIqsr = tmpINV->IqsrRef - tmpINV->Iqsr;
    tmpINV->VdsrRefInteg += Tsamp * tmpINV->KidCc * (tmpINV->ErrIdsr - tmpINV->KadCc * tmpINV->VdsrRefAw);
    tmpINV->VqsrRefInteg += Tsamp * tmpINV->KiqCc * (tmpINV->ErrIqsr - tmpINV->KaqCc * tmpINV->VqsrRefAw);

    tmpINV->VdsrRefFF = -tmpINV->Wr * tmpINV->Lq * tmpINV->IqsrRef;
    tmpINV->VqsrRefFF = tmpINV->Wr * (tmpINV->Ld * tmpINV->IdsrRef + tmpINV->Lamf);

    tmpINV->VdsrRef = tmpINV->KpdCc * tmpINV->ErrIdsr + tmpINV->VdsrRefInteg + tmpINV->VdsrRefFF;
    tmpINV->VqsrRef = tmpINV->KpqCc * tmpINV->ErrIqsr + tmpINV->VqsrRefInteg + tmpINV->VqsrRefFF;
}

void VoltageModulation(INV *tmpINV) {
    tmpINV->VdssRef = tmpINV->CosThetarComp * tmpINV->VdsrRef - tmpINV->SinThetarComp * tmpINV->VqsrRef;        // 회전좌표계 dq축 전압을 정지좌표계로 전환
    tmpINV->VqssRef = tmpINV->SinThetarComp * tmpINV->VdsrRef + tmpINV->CosThetarComp * tmpINV->VqsrRef;

    tmpINV->VasRef = tmpINV->VdssRef;                                                                           // dq를 abc로 전환
    tmpINV->VbsRef = -0.5 * (tmpINV->VdssRef - SQRT3 * tmpINV->VqssRef);
    tmpINV->VcsRef = -0.5 * (tmpINV->VdssRef + SQRT3 * tmpINV->VqssRef);

    tmpINV->VabcsRefMax = MAX(MAX(tmpINV->VasRef, tmpINV->VbsRef), tmpINV->VcsRef);                             // 3상 전압 중 최대치 판별
    tmpINV->VabcsRefMin = MIN(MIN(tmpINV->VasRef, tmpINV->VbsRef), tmpINV->VcsRef);                             // 3상 전압 중 최소치 판별

    tmpINV->VOffset = -0.5 * (tmpINV->VabcsRefMax + tmpINV->VabcsRefMin);                                       // 최대치, 최소치를 기준으로 옵셋 전압 계산

    tmpINV->VanRef = tmpINV->VasRef + tmpINV->VOffset;                                                          // 폴 전압 = 상전압 + 옵셋 전압
    tmpINV->VbnRef = tmpINV->VbsRef + tmpINV->VOffset;
    tmpINV->VcnRef = tmpINV->VcsRef + tmpINV->VOffset;

    tmpINV->DutyA = LIMIT(tmpINV->VanRef * tmpINV->InvVdc + 0.5, 0., 1.);                                       // 불가능한 지령에 대하여 듀티 범위 제한 동작
    tmpINV->DutyB = LIMIT(tmpINV->VbnRef * tmpINV->InvVdc + 0.5, 0., 1.);
    tmpINV->DutyC = LIMIT(tmpINV->VcnRef * tmpINV->InvVdc + 0.5, 0., 1.);

    tmpINV->VanOut = (tmpINV->DutyA - 0.5) * tmpINV->Vdc;                                                       // 듀티 값을 이용하여 폴 전압 결정
    tmpINV->VbnOut = (tmpINV->DutyB - 0.5) * tmpINV->Vdc;
    tmpINV->VcnOut = (tmpINV->DutyC - 0.5) * tmpINV->Vdc;

    tmpINV->VdssOut = (2. * tmpINV->VanOut - tmpINV->VbnOut - tmpINV->VcnOut) * INV3;                           // abc를 dq로 변환
    tmpINV->VqssOut = (tmpINV->VbnOut - tmpINV->VcnOut) * INV_SQRT3;

    tmpINV->VdsrOut = tmpINV->VdssOut * tmpINV->CosThetarComp + tmpINV->VqssOut * tmpINV->SinThetarComp;        // 정지좌표계 dq축 전압을 회전좌표계로 전환
    tmpINV->VqsrOut = -tmpINV->VdssOut * tmpINV->SinThetarComp + tmpINV->VqssOut * tmpINV->CosThetarComp;

    tmpINV->VdqsrOutMag = sqrt(tmpINV->VdsrOut * tmpINV->VdsrOut + tmpINV->VqsrOut * tmpINV->VqsrOut);
}

void PwmUpdate(float DutyA, float DutyB, float DutyC) {                                                         // 듀티 오입력으로 compare값이 잘못 들어가면, 범위 내로 보정한 다음 레지스터에 기록
    CmprTemp = (int16)(maxCount_samp * DutyA);
    CmprTemp = (CmprTemp > maxCount_samp) ? maxCount_samp : ((CmprTemp < 0) ? 0 : CmprTemp);
    EPwm1Regs.CMPA.half.CMPA = CmprTemp;

    CmprTemp = (int16)(maxCount_samp * DutyB);
    CmprTemp = (CmprTemp > maxCount_samp) ? maxCount_samp : ((CmprTemp < 0) ? 0 : CmprTemp);
    EPwm2Regs.CMPA.half.CMPA = CmprTemp;

    CmprTemp = (int16)(maxCount_samp * DutyC);
    CmprTemp = (CmprTemp > maxCount_samp) ? maxCount_samp : ((CmprTemp < 0) ? 0 : CmprTemp);
    EPwm3Regs.CMPA.half.CMPA = CmprTemp;
}


void Switch_DSP_On(void)
{
    if(INV_M.INVOnOffState == INV_OFF_STATE) {
        INV_M.INVOnOffState = INV_ON_STATE;

        EPwm1Regs.CMPA.half.CMPA = 0;
        EPwm2Regs.CMPA.half.CMPA = 0;
        EPwm3Regs.CMPA.half.CMPA = 0;

        EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

        EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;

        EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;

        PWM123_ENABLE;
    }
}

void Switch_DSP_Off(void)
{
    if(INV_M.INVOnOffState == INV_ON_STATE) {
        PWM123_DISABLE;
        INV_M.INVOnOffState = INV_OFF_STATE;

        EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
        EPwm1Regs.CMPA.half.CMPA = 0;
        EPwm1Regs.CMPB = 0;
        EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
        EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
        EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;
        EPwm1Regs.AQSFRC.bit.OTSFA = 1;
        EPwm1Regs.AQSFRC.bit.OTSFB = 1;
        EPwm1Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
        EPwm1Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
        EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

        EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
        EPwm2Regs.CMPA.half.CMPA = 0;
        EPwm2Regs.CMPB = 0;
        EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
        EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
        EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;
        EPwm2Regs.AQSFRC.bit.OTSFA = 1;
        EPwm2Regs.AQSFRC.bit.OTSFB = 1;
        EPwm2Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
        EPwm2Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
        EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;

        EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
        EPwm3Regs.CMPA.half.CMPA = 0;
        EPwm3Regs.CMPB = 0;
        EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
        EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
        EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
        EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;
        EPwm3Regs.AQSFRC.bit.OTSFA = 1;
        EPwm3Regs.AQSFRC.bit.OTSFB = 1;
        EPwm3Regs.AQSFRC.bit.ACTSFA = AQ_CLEAR;
        EPwm3Regs.AQSFRC.bit.ACTSFB = AQ_CLEAR;
        EPwm3Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;
    }
}
/*----------------------------------------------------------------------*/
/*         End of VARIABLE.C											*/
/*----------------------------------------------------------------------*/

