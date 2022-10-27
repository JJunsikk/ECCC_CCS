/***************************************************************
    easy28x_gen2_bitfield.h
    v9.2 (Apr 2020) :   separated from easy28x_bitfield_v9.1.h
                        supports F2802x, F2802x0, F2803x, F2805x, F2806x
    v9.3 (May 2020) :   supports F280x, F281x and F28044
    v9.4 (Sep 2020) :   supports one time reading for 4B/8B data
****************************************************************/
#ifndef _EASY28X_GEN2_BITFIELD_H__
#define _EASY28X_GEN2_BITFIELD_H__

extern void easyDSP_SCI_Init(void);
extern __interrupt void easy_RXINT_ISR(void);
extern __interrupt void easy_TXINT_ISR(void);       // for low FIFO level MCU
extern void easyDSP_SPI_Flashrom_Init(void);        // only for C2834x

// internal function declaration
extern inline void AddRing(unsigned char value);
extern char ExtractRing(void);
extern inline int IsRingEmpty(void);

/////////////////////////////////////////////////////////////////////////////////////////
// NOTICE : Please select or modify below MCU type, CPU_CLK, LSP_CLK, BAUDRATE,
//          interrupt nesting, ram function activation according to your target system
///////////////////////////////////////////////////////////////////////////////////////////
// select target MCU. only one.
// 1 = selected, 0 = not selected
#define F280x               0
#define F28044              0
#define F281x               0
#define F2802x              0
#define F2802x0             0
#define F2803x              0
#define F2805x              0
#define F2806x              0
#define F2833x              0
#define F2823x              0
#define C2834x              1
////////////////////////////////////////////////////////////////////////////////////////////
//#define CPU_CLK             50000000L       // ex. 2802x0
//#define CPU_CLK             60000000L       // ex. 2802x/3x/5x, 28015/16
//#define CPU_CLK             90000000L       // ex. 2806x
//#define CPU_CLK             100000000L      // ex. 280x, 28044
//#define CPU_CLK             150000000L      // ex. 2833x, 281x
#define CPU_CLK           300000000L      // ex. 2834x
////////////////////////////////////////////////////////////////////////////////////////////
#define LSP_CLK             (CPU_CLK/2)     // NOTE : LSP_CLK should be same to CPU_CLK in MotorWare¢â
////////////////////////////////////////////////////////////////////////////////////////////
#define BAUDRATE            115200L
////////////////////////////////////////////////////////////////////////////////////////////
// interrupt nesting assuming easyDSP ISR has the lowest priority. If not, please change the code accordingly
#define INT_NESTING_START   {           \
    IER |= 0x0100;                      \
    PieCtrlRegs.PIEACK.all = 0xFFFF;    \
    asm("       NOP");                  \
    EINT;                               \
    }
#define INT_NESTING_END         DINT
//#define INT_NESTING_START                                                  // in case of no nesting
//#define INT_NESTING_END       (PieCtrlRegs.PIEACK.all = PIEACK_GROUP9)     // in case of no nesting
////////////////////////////////////////////////////////////////////////////////////////////
// if _FLASH is not predefined by CCS configuration, you can do it here
#ifndef _FLASH
//#define _FLASH
#endif
////////////////////////////////////////////////////////////////////////////////////////////
// uncomment #pragma if fast SCI ISR run on the ram is required
#ifdef _FLASH
//#pragma CODE_SECTION(easy_RXINT_ISR, "ramfuncs");
//#pragma CODE_SECTION(AddRing, "ramfuncs");
#if F2802x || F2802x0 || F2803x || F2805x || F2806x
//#pragma CODE_SECTION(easy_TXINT_ISR, "ramfuncs");
//#pragma CODE_SECTION(ExtractRing,"ramfuncs");
#endif
#endif

#endif  // of _EASY28X_GEN2_BITFIELD_H__
