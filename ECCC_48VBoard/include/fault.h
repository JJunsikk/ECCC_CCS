#include "DSP28x_Project.h"

#ifndef DSP2834x_FLT_
#define DSP2834x_FLT_

typedef struct Fault_values
{
	float Ia_fault;
	float Ib_fault;
	float Ic_fault;
	float Idse_fault;
	float Iqse_fault;
	float Vdc_fault;
	float Wrpm_fault;
} Fault_info;

extern Uint16 TZFault, SWFault;

extern void InitFault(void);
extern void SWFaultFunc(void);
extern void ClearFault(void);
extern interrupt void FaultInt(void);


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* #ifndef _FLT_ */
