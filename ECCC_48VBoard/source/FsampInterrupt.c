#include "DSP28x_Project.h"
#include "filter.h"
#include "fault.h"
#include "variable.h"
#include "comp.h"

Uint16 FsampCnt = 0;
long FsampIntTimerStart = 0;
float TimeFsampInt = 0.;

interrupt void FsampPeakInterrupt(void) {
    FsampIntTimerStart = ReadCpuTimer0Counter();                        // ���ͷ�Ʈ ���� ���� ���� �ý��� �ð� ���

    FsampCnt++;                                                         // �ڵ尡 �� ���ư��� Ȯ���ϴ� ī����

    if(Flag.RESET == 1) {
        Flag.START_M = 0;
        Flag.RESET = 0;

        if(Flag.FAULT_CLEAR == 1)        ClearFault();

        Init_Controller(&INV_M);
    }

    ////////////////////////////// ADC Process //////////////////////////////
    ADC0_Process();                                                     // ADC
    //INV_M.Ia = ScaleAD0[4]* ((float) AD0CH[4] - OffsetAD0[4]);          // c�� ���� ����
    INV_M.Ic = ScaleAD0[0]* ((float) AD0CH[0] - OffsetAD0[0]);
    //INV_M.Ib = ScaleAD0[5]* ((float) AD0CH[5] - OffsetAD0[5]);          // b�� ���� ����
    INV_M.Ib = ScaleAD0[2]* ((float) AD0CH[2] - OffsetAD0[2]);
    //INV_M.Ic = ScaleAD0[3]* ((float) AD0CH[3] - OffsetAD0[3]);          // a�� ���� ���
    INV_M.Ia = -(INV_M.Ic + INV_M.Ib);
    INV_M.Vdc = ScaleAD0[3]* (float) AD0CH[3];                          // ������ ���� ����

    INV_M.Thetarm = BOUND_PI(INV_M.Enc_scale * (float)EQep1Regs.QPOSCNT - INV_M.ThetarmOffset);         // +PI ~ -PI ���� ������ ����

    if(INV_M.Vdc < 1.)      INV_M.InvVdc = 1.;                          // Vdc�� 1���� ���� ��, ���� �����÷ο� �߻� ����. ���� 1�� �����Ѵ�.
    else                    INV_M.InvVdc = 1. / INV_M.Vdc;              // Vdc�� ����

    if((ABS(INV_M.Ia) >= 60.) || (ABS(INV_M.Ib) >= 60.) || (ABS(INV_M.Ic) >= 60.) || (ABS(INV_M.Vdc) >= 50.) || ABS(INV_M.Wrpm) >= 3500.)
        SWFaultFunc();                                                  // ���ġ �̻��� ���� �ϳ��� �߻����� ���, ����Ʈ������ fault �߻�.

    INV_M.Idss = INV_M.Ia;                                              // d�� ���� ��ȯ
    INV_M.Iqss = (INV_M.Ib - INV_M.Ic) * INV_SQRT3;                     // q�� ���� ��ȯ

    ////////////////////////////// State machine //////////////////////////////
    INV_M.PrevState = INV_M.CurrState;
    INV_M.CurrState = INV_M.NextState;

    switch (INV_M.CurrState) {                                          // ���� �ý��� ���¸� ������� ����ġ�� ����
    case IDLE_STATE:

        if(INV_M.PrevState != IDLE_STATE) {                             // ���� ���°� IDLE ��尡 �ƴϾ�����, �� ������̳� ���� ���̾�����.
            Switch_DSP_Off();                                           // IDLE ��忡 �ִ� ���� DSP�� ����Ī�� ����� �ʱ�ȭ�� ��� �����Ѵ�.
            Init_Controller(&INV_M);
        }

        if(Flag.START_M == 1) {                                         // ���� ������ �����Ѵٴ� �÷��װ� ��������
            BootstrapCharge();                                          // ��Ʈ��Ʈ�� ĸ�� �����ϱ� ���� �ο���̵� ����ġ �� ����
            if(INV_M.BootStrapEnd == 1) INV_M.NextState = ALIGN_STATE;  // ������ �Ϸ�Ǿ�����, ���� �������� ����� ����
        }
        else {
            Switch_DSP_Off();                                           // ���� ���� ���°� �ƴϸ� PWM OFF ����
            INV_M.NextState = IDLE_STATE;
        }

        break;

    case ALIGN_STATE:
        Align(&INV_M);
        CurrentControl(&INV_M);
        VoltageModulation(&INV_M);
        PwmUpdate(INV_M.DutyA, INV_M.DutyB, INV_M.DutyC);               // VoltageModulation �Լ����� ���� ��Ƽ �������� PWM ����

        if(Flag.START_M == 0)           INV_M.NextState = IDLE_STATE;
        else if(INV_M.AlignEnd == 1)    INV_M.NextState = RUN_STATE;
        else                            INV_M.NextState = ALIGN_STATE;

        Switch_DSP_On();

        break;

    case RUN_STATE:
        SpeedObserver(&INV_M);
        CurrentRefGenerator(&INV_M);                                    // ���� ���� ������
        CurrentControl(&INV_M);                                         // ���� �����
        VoltageModulation(&INV_M);                                      // ���� ����⿡�� ���� ������ǥ�� dq�� ���� ������ ȸ����ǥ�� abc�� �������� ��ȯ(�ɼ����� �������)
        PwmUpdate(INV_M.DutyA, INV_M.DutyB, INV_M.DutyC);

        if(Flag.START_M == 0)           INV_M.NextState = IDLE_STATE;
        else                            INV_M.NextState = RUN_STATE;

        Switch_DSP_On();
        break;

    default:
        INV_M.NextState = FAULT_STATE;                                  // 0, 1, 2 �̿��� ���°� �ԷµǾ��� ��� FAULT_STATE ����
        break;
    }

    if((INV_M.NextState != FAULT_STATE) && (INV_M.Vdc < 1.))  INV_M.NextState = IDLE_STATE;     // FAULT_STATE�� �ƴѵ� ������ 1 �̸� : �ý����� �����ε� DC������ �ΰ����� �ʾҴ�.

    SerialDacOut();

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;                             // ���ͷ�Ʈ 2, 3�� ����ϹǷ�, �ش� ���ͷ�Ʈ�� �߻��� ������ �� �ֵ��� ����
    EPwm1Regs.ETCLR.bit.INT = 1;                                        // ���ͷ�Ʈ 1�ֱⰡ ������ ���� �߻��ߴ� ��� �÷��׸� �ʱ�ȭ

    TimeFsampInt = (float)(FsampIntTimerStart - ReadCpuTimer0Counter())*SYS_CLK_PRD;    // ������ �Ϸ�� ������, �� ���ͷ�Ʈ�� �����ϴ� �� �ɸ� �ð� ���
}
