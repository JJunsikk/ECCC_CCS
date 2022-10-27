#include "filter.h"

void initiateIIR1(IIR1 *p_gIIR, int type, float w0, float Ts)
{
	float a0, b0, b1;
	float INV_alpha;

	// Continuous-time Filter Coefficients
	p_gIIR->w0 = w0;
	p_gIIR->type = type;
	p_gIIR->delT = Ts;

	a0 = w0;

	switch (type)
	{
	case K_LPF:
		b0 = w0;
		b1 = 0.;
		break;
	case K_HPF:
		b0 = 0.;
		b1 = 1.;
		break;
	default:
	case K_ALLPASS:
		b0 = w0;
		b1 = -1.;
	}

	// Discrete-time Filter Coefficients
	INV_alpha = 1. / (2. + Ts*a0);
	p_gIIR->coeff[0] = (2.*b1 + Ts*b0)*INV_alpha;
	p_gIIR->coeff[1] = (-2.*b1 + Ts*b0)*INV_alpha;
	p_gIIR->coeff[2] = (2. - Ts*a0)*INV_alpha;
	p_gIIR->reg = 0.;
}

float IIR1Update(IIR1 *p_gIIR, float x)
{
	float y;

	y = p_gIIR->reg + p_gIIR->coeff[0] * x;
	p_gIIR->reg = p_gIIR->coeff[1] * x + p_gIIR->coeff[2] * y;

	return(y);
}

void initiateIIR2(IIR2 *p_gIIR, int type, float w0, float zeta, float Ts)
{
	float a0, a1, b0, b1, b2;
	float INV_alpha;

	// Continuous-time Filter Coefficients
	p_gIIR->w0 = w0;
	p_gIIR->zeta = zeta;
	p_gIIR->delT = Ts;
	p_gIIR->type = type;

	a0 = w0*w0;
	a1 = 2. * zeta*w0;
	switch (type)
	{
	case K_LPF:
		b0 = w0*w0;
		b1 = 0.;
		b2 = 0.;
		break;
	case K_HPF:
		b0 = 0.;
		b1 = 0.;
		b2 = 1.;
		break;
	case K_BPF:
		b0 = 0.;
		b1 = 2. * zeta*w0;
		b2 = 0.;
		break;
	case K_NOTCH:
		b0 = w0*w0;
		b1 = 0.;
		b2 = 1.;
		break;
	case K_ALLPASS:
	default:
		b0 = w0*w0;
		b1 = -2. * zeta*w0;
		b2 = 1.;
	}

	// Discrete-time Filter Coefficients
	INV_alpha = 1. / (4. + 2.*Ts*a1 + Ts*Ts*a0);
	p_gIIR->coeff[0] = (4. * b2 + 2.*Ts*b1 + Ts*Ts*b0)*INV_alpha;
	p_gIIR->coeff[1] = (2. * Ts*Ts*b0 - 8.*b2)*INV_alpha;
	p_gIIR->coeff[2] = -(2. * Ts*Ts*a0 - 8.)*INV_alpha;
	p_gIIR->coeff[3] = (4.*b2 - 2.*Ts*b1 + Ts*Ts*b0)*INV_alpha;
	p_gIIR->coeff[4] = -(4. - 2.*Ts*a1 + Ts*Ts*a0)*INV_alpha;

	p_gIIR->reg[0] = 0.;
	p_gIIR->reg[1] = 0.;

}

float IIR2Update(IIR2 *p_gIIR, const float x)
{
	float y;

	y = p_gIIR->reg[0] + p_gIIR->coeff[0] * x;
	p_gIIR->reg[0] = p_gIIR->reg[1] + p_gIIR->coeff[1] * x + p_gIIR->coeff[2] * y;
	p_gIIR->reg[1] = p_gIIR->coeff[3] * x + p_gIIR->coeff[4] * y;

	return(y);
}
