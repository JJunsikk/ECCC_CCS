#include "DSP28x_Project.h"
#include "filter.h"
#include "fault.h"
#include "variable.h"
#include "comp.h"

Uint16 FsampCnt = 0;
long FsampIntTimerStart = 0;
float TimeFsampInt = 0.;

interrupt void FsampPeakInterrupt(void) {
    FsampIntTimerStart = ReadCpuTimer0Counter();                        // 인터럽트 동작 시작 시의 시스템 시간 기록

    FsampCnt++;                                                         // 코드가 잘 돌아가나 확인하는 카운터

    if(Flag.RESET == 1) {
        Flag.START_M = 0;
        Flag.RESET = 0;

        if(Flag.FAULT_CLEAR == 1)        ClearFault();

        Init_Controller(&INV_M);
    }

    ////////////////////////////// ADC Process //////////////////////////////
    ADC0_Process();                                                     // ADC
    //INV_M.Ia = ScaleAD0[4]* ((float) AD0CH[4] - OffsetAD0[4]);          // c상 전류 센싱
    INV_M.Ic = ScaleAD0[0]* ((float) AD0CH[0] - OffsetAD0[0]);
    //INV_M.Ib = ScaleAD0[5]* ((float) AD0CH[5] - OffsetAD0[5]);          // b상 전류 센싱
    INV_M.Ib = ScaleAD0[2]* ((float) AD0CH[2] - OffsetAD0[2]);
    //INV_M.Ic = ScaleAD0[3]* ((float) AD0CH[3] - OffsetAD0[3]);          // a상 전류 계산
    INV_M.Ia = -(INV_M.Ic + INV_M.Ib);
    INV_M.Vdc = ScaleAD0[3]* (float) AD0CH[3];                          // 직류단 전압 센싱

    INV_M.Thetarm = BOUND_PI(INV_M.Enc_scale * (float)EQep1Regs.QPOSCNT - INV_M.ThetarmOffset);         // +PI ~ -PI 범위 값으로 가공

    if(INV_M.Vdc < 1.)      INV_M.InvVdc = 1.;                          // Vdc가 1보다 작을 때, 값의 오버플로우 발생 가능. 따라서 1로 고정한다.
    else                    INV_M.InvVdc = 1. / INV_M.Vdc;              // Vdc의 역수

    if((ABS(INV_M.Ia) >= 60.) || (ABS(INV_M.Ib) >= 60.) || (ABS(INV_M.Ic) >= 60.) || (ABS(INV_M.Vdc) >= 50.) || ABS(INV_M.Wrpm) >= 3500.)
        SWFaultFunc();                                                  // 허용치 이상의 값이 하나라도 발생했을 경우, 소프트웨어적 fault 발생.

    INV_M.Idss = INV_M.Ia;                                              // d축 전류 변환
    INV_M.Iqss = (INV_M.Ib - INV_M.Ic) * INV_SQRT3;                     // q축 전류 변환

    ////////////////////////////// State machine //////////////////////////////
    INV_M.PrevState = INV_M.CurrState;
    INV_M.CurrState = INV_M.NextState;

    switch (INV_M.CurrState) {                                          // 현재 시스템 상태를 기반으로 스위치문 수행
    case IDLE_STATE:

        if(INV_M.PrevState != IDLE_STATE) {                             // 이전 상태가 IDLE 모드가 아니었으면, 즉 얼라인이나 운전 중이었으면.
            Switch_DSP_Off();                                           // IDLE 모드에 있는 동안 DSP의 스위칭과 제어기 초기화를 계속 수행한다.
            Init_Controller(&INV_M);
        }

        if(Flag.START_M == 1) {                                         // 모터 운전을 시작한다는 플래그가 세워지면
            BootstrapCharge();                                          // 부트스트랩 캡을 충전하기 위한 로우사이드 스위치 온 로직
            if(INV_M.BootStrapEnd == 1) INV_M.NextState = ALIGN_STATE;  // 충전이 완료되었으면, 다음 스텝으로 얼라인 수행
        }
        else {
            Switch_DSP_Off();                                           // 모터 운전 상태가 아니면 PWM OFF 유지
            INV_M.NextState = IDLE_STATE;
        }

        break;

    case ALIGN_STATE:
        Align(&INV_M);
        CurrentControl(&INV_M);
        VoltageModulation(&INV_M);
        PwmUpdate(INV_M.DutyA, INV_M.DutyB, INV_M.DutyC);               // VoltageModulation 함수에서 만든 듀티 지령으로 PWM 생성

        if(Flag.START_M == 0)           INV_M.NextState = IDLE_STATE;
        else if(INV_M.AlignEnd == 1)    INV_M.NextState = RUN_STATE;
        else                            INV_M.NextState = ALIGN_STATE;

        Switch_DSP_On();

        break;

    case RUN_STATE:
        SpeedObserver(&INV_M);
        CurrentRefGenerator(&INV_M);                                    // 전류 지령 생성기
        CurrentControl(&INV_M);                                         // 전류 제어기
        VoltageModulation(&INV_M);                                      // 전류 제어기에서 나온 정지좌표계 dq축 전압 지령을 회전좌표계 abc축 지령으로 전환(옵셋전압 변조방식)
        PwmUpdate(INV_M.DutyA, INV_M.DutyB, INV_M.DutyC);

        if(Flag.START_M == 0)           INV_M.NextState = IDLE_STATE;
        else                            INV_M.NextState = RUN_STATE;

        Switch_DSP_On();
        break;

    default:
        INV_M.NextState = FAULT_STATE;                                  // 0, 1, 2 이외의 상태가 입력되었을 경우 FAULT_STATE 진입
        break;
    }

    if((INV_M.NextState != FAULT_STATE) && (INV_M.Vdc < 1.))  INV_M.NextState = IDLE_STATE;     // FAULT_STATE는 아닌데 전압이 1 미만 : 시스템은 정상인데 DC전원이 인가되지 않았다.

    SerialDacOut();

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;                             // 인터럽트 2, 3을 사용하므로, 해당 인터럽트의 발생을 감지할 수 있도록 설정
    EPwm1Regs.ETCLR.bit.INT = 1;                                        // 인터럽트 1주기가 끝나기 전에 발생했던 모든 플래그를 초기화

    TimeFsampInt = (float)(FsampIntTimerStart - ReadCpuTimer0Counter())*SYS_CLK_PRD;    // 동작이 완료된 시점에, 한 인터럽트를 수행하는 데 걸린 시간 계산
}
