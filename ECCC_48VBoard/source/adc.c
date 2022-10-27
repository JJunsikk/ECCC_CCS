#include "variable.h"
#include "comp.h"
Uint16 uOffsetLoopCnt = 0, uOffsetMaxCnt = (1<<14);
Uint16 uOffset_v = 0, uCnt = 0;
Uint16 AD0CH[6] = {0., 0., 0., 0., 0., 0.};
float OffsetAD0[6] = { 0. , 0., 0., 0., 0., 0.};

float ScaleAD0[6] = {
         1. / 0.5e-3 / 93. * 5. / 4096.,              //CHA0+ : Idc
         (3.9e3 * 3. + 1.e3) / 1.e3 * 5. / 4096.,    //CHB0+ : Vdc
         1. / 0.5e-3 / 93. * 5. / 4096.,	          //CHA1- : IdcFilt
         1. / 30e-3 * 5. / 4096.,                     //CHB1- : Ic
         -1. / 30e-3 * 5. / 4096.,	                  //CHA1+ : Ia
         1. / 30e-3 * 5. / 4096.,                     //CHB1+ : Ib

};

void InitAdc(void)
{
	AD0_RD = 0x101;
	AD0_RD = 0x03FF;
}

void ADC0_Process(void)
{
    AD0_RD = 0x0100; // CHA0+, CHB0+
    delaycc(5e-9);
    ADC0_SOC_START;
    delaycc(5e-9);
    ADC0_SOC_END;
    while(ADC0_BUSY);
    AD0CH[0] = (AD0_RD - 0x0800) & 0x0FFF;
    AD0CH[1] = (AD0_RD - 0x0800) & 0x0FFF;

	AD0_RD = 0x0500; // CHA1-, CHB1-
	delaycc(5e-9);
	ADC0_SOC_START;
	delaycc(5e-9);
	ADC0_SOC_END;
	while(ADC0_BUSY);
	AD0CH[2] = (AD0_RD - 0x0800) & 0x0FFF;
	AD0CH[3] = (AD0_RD - 0x0800) & 0x0FFF;

    AD0_RD = 0x0900; // CHA1+, CHB1+
    delaycc(5e-9);
    ADC0_SOC_START;
    delaycc(5e-9);
    ADC0_SOC_END;
    while(ADC0_BUSY);
    AD0CH[4] = (AD0_RD - 0x0800) & 0x0FFF;
    AD0CH[5] = (AD0_RD - 0x0800) & 0x0FFF;
}

interrupt void OffsetInterrupt(void)
{
    uOffsetLoopCnt++;

    if (!uOffset_v)
    {
        if (uOffsetLoopCnt > 99)
        {
            uOffset_v = 1;
            uOffsetLoopCnt = 0 ;
        }
    }
    else
      ADCOffset();

    if(uOffsetLoopCnt >= uOffsetMaxCnt)
    {
        for (uCnt = 0; uCnt < 6; uCnt++) {
            OffsetAD0[uCnt] = OffsetAD0[uCnt]/(float)uOffsetMaxCnt;
        }

        EALLOW;
        PieVectTable.EPWM1_INT = &FsampPeakInterrupt;
        EDIS;
    }

    EINT;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    EPwm1Regs.ETCLR.bit.INT = 1;
}

void ADCOffset(void)
{
	AD0_RD = 0x0100;
	delaycc(5e-9);
	ADC0_SOC_START;
	delaycc(5e-9);
	ADC0_SOC_END;
	while(ADC0_BUSY);
	OffsetAD0[0] += (AD0_RD - 0x0800) & 0x0FFF;
	OffsetAD0[1] += (AD0_RD - 0x0800) & 0x0FFF;

	AD0_RD = 0x0500;
	delaycc(5e-9);
	ADC0_SOC_START;
	delaycc(5e-9);
	ADC0_SOC_END;
	while(ADC0_BUSY);
	OffsetAD0[2] += (AD0_RD - 0x0800) & 0x0FFF;
	OffsetAD0[3] += (AD0_RD - 0x0800) & 0x0FFF;

    AD0_RD = 0x0900;
    delaycc(5e-9);
    ADC0_SOC_START;
    delaycc(5e-9);
    ADC0_SOC_END;
    while(ADC0_BUSY);
    OffsetAD0[4] += (AD0_RD - 0x0800) & 0x0FFF;
    OffsetAD0[5] += (AD0_RD - 0x0800) & 0x0FFF;
}
//===========================================================================
// End of file.
//===========================================================================
