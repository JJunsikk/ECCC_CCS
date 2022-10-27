#ifndef	DSP2834x__COMP
#define	DSP2834x__COMP

//Triangular Function constant
#define f2      	((float)0.5)
#define	f3			((float)0.16666666666666666666666666666667)
#define	f4			((float)0.04166666666666666666666666666667)
#define	f5			((float)0.00833333333333333333333333333333)
#define	f6			((float)0.00138888888888888888888888888889)
#define	f7			((float)1.9841269841269841269841269841e-4)
#define	f8			((float)2.480158730158730158730158730125e-5)
#define	f9			((float)2.75573192239858906525573192e-6)
#define	f10			((float)2.7557319223985890652557319e-7)
#define	f11			((float)2.505210838544171877505211e-8)
#define	f12			((float)2.08767569878680989792101e-9)
#define	f13			((float)1.6059043836821614599392e-10)
#define	f14			((float)1.147074559772972471385e-11)
#define	f15			((float)7.6471637318198164759e-13)

// floating-point Constants
#define	PI          ((float)3.1415926535897932384626433832795)
#define	PI2			((float)6.283185307179586476925286766559)
#define	SQRT2		((float)1.4142135623730950488016887242097)
#define	SQRT3		((float)1.7320508075688772935274463415059)
#define INV3        ((float)0.3333333333333333333333333333333)
#define	INV_SQRT3	((float)0.57735026918962576450914878050196)
#define	INV_SQRT2	((float)0.70710678118654752440084436210485)
#define	SQRT3HALF   ((float)0.86602540378443864676372317075294)
#define INV_PI		((float)0.31830988618379067153776752674503)
#define INV_2PI     ((float)0.15915494309189533576888376337251)
#define PIBY3       ((float)1.0471975511965977461542144610932)
#define INV_PIBY3   ((float)0.9549296585513720146133025802350)

#define RPM2RM      ((float)0.1047197551196597746154214461093)
#define RM2RPM      ((float)9.5492965855137201461330258023509)

// Macro Functions
#define LIMIT(x,s,l)			(((x)>(l))?(l):((x)<(s))?((s)):(x))
#define	MAX(a, b)				((a)>(b) ? (a) : (b))
#define	MIN(a, b)				((a)>(b) ? (b) : (a))
#define	BOUND_PI(x)             (((x)>0)?((x)-PI2*(int)(((x)+PI)*INV_2PI)):((x)-PI2*(int)(((x)-PI)*INV_2PI)))
#define	ABS(x)					(((x)>0)?(x):(-(x)))
#define	SIGN(x)					(((x)<0)? -1. : 1. )
#define SIN(x,x2)		        ((x)*(1.-(x2)*(f3-(x2)*(f5-(x2)*(f7-(x2)*(f9-(x2)*(f11-(x2)*(f13-(x2)*f15))))))))
#define COS(x2)			        (1.-(x2)*(f2-(x2)*(f4-(x2)*(f6-(x2)*(f8-(x2)*(f10-(x2)*(f12-(x2)*f14)))))))

#endif
