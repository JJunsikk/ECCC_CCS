#include "DSP28x_Project.h"
#include "filter.h"

#ifndef	DSP2834x_VARIABLE_H
#define	DSP2834x_VARIABLE_H

////////////////////////////// MACRO SET //////////////////////////////
#define SYS_CLK	    300.e6
#define SYS_CLK_PRD	3.3333333333333333333333333333333e-9
#define PWM_CLK     300.e6
#define PWM_CLK_PRD 3.3333333333333333333333333333333e-9
#define Fsamp       20.e3
#define Fsc         1.e3
#define Tdead       1e-6

#define XINTF_ZONE7         0x200000
#define AD_CS_BASE          (XINTF_ZONE7 + 0x1000)
#define AD0_RD              (*(volatile int *)(AD_CS_BASE + 0x0000))
#define AD1_RD              (*(volatile int *)(AD_CS_BASE + 0x0002))

#define ADC0_SOC_START		GpioDataRegs.GPBCLEAR.bit.GPIO46 = 1
#define ADC0_SOC_END		GpioDataRegs.GPBSET.bit.GPIO46 = 1
#define ADC0_BUSY			GpioDataRegs.GPBDAT.bit.GPIO47

#define PWM123_ENABLE       GpioDataRegs.GPACLEAR.bit.GPIO24 = 1
#define PWM123_DISABLE      GpioDataRegs.GPASET.bit.GPIO24 = 1

// State machine
#define IDLE_STATE          0
#define ALIGN_STATE         1
#define RUN_STATE           2
#define FAULT_STATE         3

// INV On/Off state
#define INV_OFF_STATE       0
#define INV_ON_STATE        1

// Control mode
#define CONST_CUR_MODE      0
#define VECTCONTL_MODE      1
#define SPDCONTL_MODE       2

// Encoder
#define ENC_PPR             1024

// PWM
extern float Tsamp, Tsc;
extern Uint16 maxCount_samp, maxCount_SC;
extern int32 CmprTemp;

// ADC
extern Uint16 AD0CH[6];
extern float OffsetAD0[6];
extern float ScaleAD0[6];

// DAC
extern long *da[4];
extern int  da_type[4], da_data[4];
extern float da_scale[4], da_mid[4], da_temp[4];

typedef struct _INV_
{
    Uint16 INVOnOffState;

    // State machine
    Uint16 PrevState, CurrState, NextState;

    // Motor parameters
    float P, PP;
    float Rs, Ld, Lq, Lamf;
    float Jm, Bm;
    float Irated;
    float InvLamf, InvLd, InvLq, InvPP, InvJm;
    float Kt, InvKt;

    // Sensing values
    float Vdc, InvVdc;
    float IdcMin, IdcMid;
    float Ia, Ib, Ic;
    float Enc_scale;
    float Thetarm;

    // BootStrap initial charge
    Uint16 BootStrapEnd, BootStrapStepCnt;
    Uint16 BootStrapChargeEpwmCnt;

    // Align
    Uint16 AlignEnd;
    Uint32 AlignStep, AlignCnt, AlignCntMax;
    float IdsrRefSetAlign, IdsrRefAlign, DelIdsrAlign;
    float WrRefSetAlign, WrRefAlign, DelWrRefAlign, ThetarAlign;
    float ThetarmOffsetTemp, ThetarmOffset;
    float INV_AlignCntPlus1;

    // Constant I mode
    float IdsrRefSetIbyF, IdsrRefIbyF, DelIdsrRefIbyF;
    float WrRefSetIbyF, WrRefIbyF, DelWrRefIbyF;
    float ThetarIbyF, ThetarIbyFSqr, ThetarCompIbyF, ThetarCompIbyFSqr;

    // Speed estimator
    float WcSO1, WcSO23;
    float ZetaSO;
    float K1, K2, K3;
    float ErrThetarm, AccEstInteg, AccFF;
    float Wrpm, Wrm, Wr;
    float InvWr;
    float ThetarmEst, ThetarEst;
    float WcSpdLPF;
    float WrmLPFilt;
    IIR1 WrmLPF;

    // Speed controller
    float WcSc;
    float KpSc, KiSc, KaSc;
    float WrpmRef, WrpmRefSet, WrmRef, DelWrpmRef;
    float ErrWrm, TeRefInteg, TeRefUnsat, TeRef, TeRefAW;
    float TeRefMin, TeRefMax;

    // Field weakening
    float KpFW, KiFW, KaFW;
    float VLimRatioFW, VdqsrRefMax;
    float VmagErr, DelIdsrRefFWInteg, DelIdsrRefFWUnsat, DelIdsrRefFW, DelIdsrRefFWAW;
    float IdsrRefMax, IqsrRefMax;

    // Current controller
    Uint16 InitControlMode;
    float Thetar, ThetarSqr;
    float ThetarComp, ThetarCompSqr;
    float CosThetar, SinThetar;
    float CosThetarComp, SinThetarComp;
    float WcCc;
    float KpdCc, KidCc, KadCc;
    float KpqCc, KiqCc, KaqCc;
    float IdsrRef, IqsrRef, IdsrRefSet, IqsrRefSet;
    float Idss, Iqss;
    float Idsr, Iqsr;
    float ErrIdsr, VdsrRefInteg, VdsrRefFF, VdsrRefAw, VdsrRef, VdsrRefTemp;
    float ErrIqsr, VqsrRefInteg, VqsrRefFF, VqsrRefAw, VqsrRef, VqsrRefTemp;
    float VdqsrOutMag;
    float VLimRatioCC;

    // Voltage modulation
    float VdssRef, VqssRef;
    float VasRef, VbsRef, VcsRef;
    float VabcsRefMax, VabcsRefMin, VOffset;
    float VanRef, VbnRef, VcnRef;
    float DutyA, DutyB, DutyC;
    float VanOut, VbnOut, VcnOut;
    float VdssOut, VqssOut;
    float VdsrOut, VqsrOut;
} INV;

// Flag
typedef struct FLAG_BITS
{
    Uint16 START_M;
    Uint16 RESET;
    Uint16 FAULT_CLEAR;
} FLAG_REG;

extern FLAG_REG Flag;
extern INV INV_M;

extern interrupt void FaultInterrupt(void);
extern interrupt void OffsetInterrupt(void);
extern interrupt void FsampPeakInterrupt(void);
extern interrupt void FscInterrupt(void);

extern void delaycc(float time);
extern void SlopeGenerator(float *Var, float Cmd, float DelPerTs);

extern void InitFlag(void);
extern void InitAdc(void);
extern void ADCOffset(void);
extern void ADC0_Process(void);
extern void InitSerialDac(void);
extern void SerialDacOut(void);

extern void InitParameter(INV *tmpINV);
extern void InitUserDefinedVars(INV *tmpINV);
extern void Init_Controller(INV *tmpINV);
extern void UpdateGain(INV *tmpINV);

extern void SpeedObserver(INV *tmpINV);
extern void CurrentRefGenerator(INV *tmpINV);
extern void CurrentControl(INV *tmpINV);
extern void VoltageModulation(INV *tmpINV);

extern void Align(INV *tmpINV);
extern void I_BY_F(INV *tmpINV);

extern void BootstrapCharge(void);
extern void PwmUpdate(float DutyA, float DutyB, float DutyC);
extern void Switch_DSP_On(void);
extern void Switch_DSP_Off(void);
#endif
