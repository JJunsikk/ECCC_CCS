#ifndef DSP2834x_FILTER__
#define DSP2834x_FILTER__

#define	K_ALLPASS	0
#define	K_LPF		1
#define	K_HPF		2
#define	K_BPF		3
#define	K_NOTCH		4

typedef struct {
	int type;
	float w0;
	float delT;
	float coeff[3], reg;
	float a0, b0, b1, INV_alpha;
}	IIR1;


typedef struct {
	int type;
	float w0, zeta;
	float delT;
	float coeff[5], reg[2];
	float a0, a1, b0, b1, b2, INV_alpha;
}	IIR2;


extern void initiateIIR1(IIR1 *p_gIIR, int type, float w0, float Ts);
extern void initiateIIR2(IIR2 *p_gIIR, int type, float w0, float zeta, float Ts);

extern float IIR1Update(IIR1 *p_gIIR, float input);
extern float IIR2Update(IIR2 *p_gIIR, const float input);
#endif

