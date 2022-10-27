/***************************************************************
    easy28x_gen2_bitfield.c
	released version at v9.4

****************************************************************/
// Included Files
#include "easy28x_gen2_BitField_v9.4.h"
#if F281x
#include "DSP281x_Device.h"
#elif F280x
#include "DSP280x_Device.h"
#elif F28044
#include "DSP2804x_Device.h"
#else
#include "DSP28x_Project.h"
#endif

//////////////////////////////////////////////////
// DON'T CHANGE THIS FILE
// 소스 파일이 아닌 헤더 파일을 변경하셔야 합니다.
//////////////////////////////////////////////////
#define IS_FIFO_16_LEVEL   (F2833x || F2823x || C2834x || F281x || F280x || F28044)

// easyDSP commands & states
#define STAT_INIT       0
#define STAT_ADDR       1
#define STAT_DATA2B     2
#define STAT_DATA4B     3
#define STAT_WRITE      4
#define STAT_DATA8B     5
#define STAT_CONFIRM    6
#define STAT_DATA_BLOCK 7
#define STAT_FLASH		8					// flash for c2834x

#define CMD_ADDR            0xE7
#define CMD_READ2B          0xDB
#define CMD_READ4B          0xC3
#define CMD_READ8B          0x8B
#define CMD_READ16B         0x28
#define CMD_DATA2B          0xBD
#define CMD_DATA4B          0x99
#define CMD_DATA8B          0x64
#define CMD_DATA_BLOCK      0x55
#define CMD_WRITE           0x7E
#define CMD_FB_READ         0x0D
#define CMD_FB_WRITE_OK     0x0D
#define CMD_FB_WRITE_NG     0x3C
#define CMD_CHANGE_CPU      0X5A
#define CMD_CONFIRM         0xA5
#define CMD_FLASH				0x96	// flash for c2834x
#define CMD_FLASH_WRITE_READ	0xF0	// flash for c2834x
#define CMD_FLASH_WRITE_ONLY	0x11	// flash for c2834x

// for internal use
unsigned int ezDSP_Version_SCI = 940;
unsigned int ezDSP_uRead16BPossible = 1, ezDSP_uRead8BPossible = 1;
float ezDSP_fFloat = 0;
unsigned int ezDSP_uOnChipFlash = 0;

void easyDSP_SCI_Init()
{
	int i;

    //SCI CLOCK ENABLE
    EALLOW;
#if F281x
    SysCtrlRegs.PCLKCR.bit.SCIAENCLK=1;
#else
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
#endif
    EDIS;

#if (F2823x || F2833x || C2834x || F2806x || F2805x || F2803x || F2802x || F2802x0 || F280x || F28044)
    EALLOW;
    // This will enable the pullups for the specified pins. as in boot rom code
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up for GPIO29 (SCITXDA)
    // This will select asynch (no qualification) for the selected pins. as in boot rom code
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
    // This specifies which of the possible GPIO pins will be SCI functional pins.
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation
    EDIS;
#elif F281x
    EALLOW;
    GpioMuxRegs.GPFMUX.bit.SCITXDA_GPIOF4 = 1;
    GpioMuxRegs.GPFMUX.bit.SCIRXDA_GPIOF5 = 1;
    EDIS;
#else
    this will create compiler error intentionally
#endif

    // configure SCI
    SciaRegs.SCICCR.bit.STOPBITS = 0;   // 1 stop
    SciaRegs.SCICCR.bit.PARITYENA = 0;  // no parity
    SciaRegs.SCICCR.bit.LOOPBKENA = 0;  // no loopback
    SciaRegs.SCICCR.bit.SCICHAR = 7;    // 8bit char
    i = (int)(((float)LSP_CLK/(BAUDRATE*8.) - 1) + 0.5);
    SciaRegs.SCIHBAUD = i >> 8;
    SciaRegs.SCILBAUD = i & 0xFF;

    // enable Module
    SciaRegs.SCICTL1.bit.SWRESET = 1;
    SciaRegs.SCICTL1.bit.TXENA = 1;
    SciaRegs.SCICTL1.bit.RXENA = 1;

    // enable FIFO
    SciaRegs.SCIFFTX.bit.SCIRST = 1;
    SciaRegs.SCIFFTX.bit.SCIFFENA = 1;
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;

    // enable FIFO interrupt
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
    //SciaRegs.SCICTL1.bit.RXERRINTENA = 1;
#if !IS_FIFO_16_LEVEL
    SciaRegs.SCIFFTX.bit.TXFFIENA = 1;
#endif

    // FIFO interrupts level
    SciaRegs.SCIFFRX.bit.RXFFIL = 1;
    //SciaRegs.SCIFFTX.bit.TXFFIL = 1;    // no need ?
#if !IS_FIFO_16_LEVEL
    SciaRegs.SCIFFTX.bit.TXFFIL = 0;
#endif

    // perform SW reset
    SciaRegs.SCICTL1.bit.SWRESET = 0;
    SciaRegs.SCICTL1.bit.SWRESET = 1;

    // reset FIFO
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 0;
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;

    // Reassign ISR for easyDSP.
	EALLOW;
#if F281x
	PieVectTable.RXAINT = &easy_RXINT_ISR;
#else
	PieVectTable.SCIRXINTA = &easy_RXINT_ISR;
#endif
#if !IS_FIFO_16_LEVEL
	PieVectTable.SCITXINTA = &easy_TXINT_ISR;
#endif
    EDIS;

	// Enable interrupts required
#if F281x
    PieCtrlRegs.PIECRTL.bit.ENPIE = 1;      // Enable the PIE block
#else
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;		// Enable the PIE block
#endif
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // Enable SCI-A RX INT in the PIE: Group 9 interrupt 1
#if !IS_FIFO_16_LEVEL
	PieCtrlRegs.PIEIER9.bit.INTx2 = 1;      // Enable SCI-A TX INT in the PIE: Group 9 interrupt 1
#endif

	IER |= M_INT9;							// Enable CPU INT9 for SCI-A  (M_INT9 = 0x100)
	EINT;									// Enable Global interrupt INTM

	// others
#ifdef _FLASH
	ezDSP_uOnChipFlash = 1;
#endif
}

// error counter
unsigned int ezDSP_uBRKDTCount = 0, ezDSP_uFECount = 0, ezDSP_uOECount = 0, ezDSP_uPECount = 0;
unsigned int ezDSP_uWrongISRCount = 0;

// for easyDSP
unsigned char ezDSP_ucRx = 0;
unsigned int ezDSP_uState = STAT_INIT, ezDSP_uData = 0, ezDSP_uChksum = 0;
unsigned long ezDSP_ulData = 0, ezDSP_ulAddr = 0;
unsigned int ezDSP_uAddrRdCnt = 0, ezDSP_uDataRdCnt = 0;
unsigned long long ezDSP_ullData = 0;

#if C2834x
// for spi flashrom /w c2834x
#define SPIFLASHERR_SPITXFULL	(0x01<<0)
#define SPIFLASHERR_SPIRXOVER	(0x01<<1)
#define SPIFLASHERR_SPIRXDELAY	(0x01<<2)
#define SPIFLASHERR_WRONGCMD	(0x01<<3)
unsigned int ezDSP_uFlashRdCnt = 0, ezDSP_uFlashErrorCount = 0;
unsigned int ezDSP_uFlashDataCount = 0, ezDSP_uLive = 0xA5A5;
unsigned char ezDSP_ucFlashCmd = 0, ezDSP_ucFlashRead = 0;
unsigned int ezDSP_uFlashErrorFlag = 0;
#endif
unsigned int ezDSP_uBlockSize = 0, ezDSP_uBlockIndex = 0, ezDSP_uChkSumCalculated = 0;
unsigned int ezDSP_uISRRxCount = 0, ezDSP_uISRTxCount = 0;
unsigned int ezDSP_uRxFifoCnt, ezDSP_uMaxRxFifoCnt = 0, ezDSP_uTxFifoCnt, ezDSP_uMaxTxFifoCnt = 0;
unsigned int ezDSP_uTxFifoFullCnt = 0;      // something wrong if not zero

#if !IS_FIFO_16_LEVEL
#define TX_FF_START     {   if(SciaRegs.SCIFFTX.bit.TXFFST < 4) {  \
                                SciaRegs.SCITXBUF = ExtractRing();  \
                                SciaRegs.SCIFFTX.bit.TXFFIENA = 1;  \
                            }                                       \
                        }
#define TX_FF_STOP      (SciaRegs.SCIFFTX.bit.TXFFIENA = 0)
interrupt void easy_TXINT_ISR(void)
{
    INT_NESTING_START;
    ezDSP_uISRTxCount++;

    if(IsRingEmpty()) {
        TX_FF_STOP;
    }
    else {
        while(!IsRingEmpty() && (SciaRegs.SCIFFTX.bit.TXFFST < 4))
            SciaRegs.SCITXBUF = ExtractRing();
        if(IsRingEmpty()) {
            TX_FF_STOP;
        }
        else
            ezDSP_uTxFifoFullCnt++;
    }

    // counting after input to Tx
    ezDSP_uTxFifoCnt = SciaRegs.SCIFFTX.bit.TXFFST;
    if(ezDSP_uTxFifoCnt > ezDSP_uMaxTxFifoCnt) ezDSP_uMaxTxFifoCnt = ezDSP_uTxFifoCnt;

    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;   // Clear Interrupt flag
    INT_NESTING_END;
}
#endif

#if IS_FIFO_16_LEVEL
inline void AddRing(unsigned char y) {
    if(SciaRegs.SCIFFTX.bit.TXFFST != 16)     SciaRegs.SCITXBUF = y;
    else                                      ezDSP_uTxFifoFullCnt++;

    // counting after input to Tx
    ezDSP_uTxFifoCnt = SciaRegs.SCIFFTX.bit.TXFFST;
    if(ezDSP_uTxFifoCnt > ezDSP_uMaxTxFifoCnt) ezDSP_uMaxTxFifoCnt = ezDSP_uTxFifoCnt;
}
#define TX_FF_START     // nop
#endif


__interrupt void easy_RXINT_ISR()
{
    INT_NESTING_START;

    ezDSP_uISRRxCount++;
	Uint16 uIndex;

	// check RX Error
	if(SciaRegs.SCIRXST.bit.RXERROR) {
		if(SciaRegs.SCIRXST.bit.BRKDT)	ezDSP_uBRKDTCount++;	// Break Down
		if(SciaRegs.SCIRXST.bit.FE) 	ezDSP_uFECount++;		// FE
		if(SciaRegs.SCIRXST.bit.OE) 	ezDSP_uOECount++;		// OE
		if(SciaRegs.SCIRXST.bit.PE)		ezDSP_uPECount++;		// PE

		// 'Break down' stops further Rx operation.
		// software reset is necessary to clear its status bit and proceed further rx operation
		SciaRegs.SCICTL1.bit.SWRESET = 0;
		SciaRegs.SCICTL1.bit.SWRESET = 1;

        //FIFO
        SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   // Clear Overflow flag
        SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag

        ezDSP_uState = STAT_INIT;

        INT_NESTING_END;
		return;
	}

#if F281x
	if(!SciaRegs.SCIFFRX.bit.RXFIFST) {
#else
    if(!SciaRegs.SCIFFRX.bit.RXFFST) {
#endif
        ezDSP_uWrongISRCount++;
        SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   // Clear Overflow flag
        SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag
        INT_NESTING_END;
        return;
    }

    // monitoring
#if F281x
    ezDSP_uRxFifoCnt = SciaRegs.SCIFFRX.bit.RXFIFST;
#else
    ezDSP_uRxFifoCnt = SciaRegs.SCIFFRX.bit.RXFFST;
#endif
    if(ezDSP_uRxFifoCnt > ezDSP_uMaxRxFifoCnt) ezDSP_uMaxRxFifoCnt = ezDSP_uRxFifoCnt;

    // FIFO
#if F281x
    while(SciaRegs.SCIFFRX.bit.RXFIFST) {
#else
    while(SciaRegs.SCIFFRX.bit.RXFFST) {
#endif
        ezDSP_ucRx = SciaRegs.SCIRXBUF.all;

        ////////////////////////////////////////////
        // Parsing by state
        ////////////////////////////////////////////
        if(ezDSP_uState == STAT_INIT) {
            if(ezDSP_ucRx == CMD_ADDR) {
                ezDSP_uState = STAT_ADDR;
                ezDSP_uAddrRdCnt = 0;
            }
            else if(ezDSP_ucRx == CMD_READ2B) {
                ezDSP_ulAddr++;	// auto increment
                ezDSP_uData = *(unsigned int*)ezDSP_ulAddr;

                AddRing(ezDSP_uData >> 8);	// MSB
                AddRing(ezDSP_uData);		// LSB
                AddRing(CMD_FB_READ);

                ezDSP_uState = STAT_INIT;
                TX_FF_START;
            }
            else if(ezDSP_ucRx == CMD_READ16B) {
                ezDSP_ulAddr += 8;
                for(uIndex = 0; uIndex < 8; uIndex++) {
                    // Since this is not for variable, addresss is increased sequentially
                    ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + uIndex);
                    AddRing(ezDSP_uData >> 8);		// MSB
                    AddRing(ezDSP_uData);			// LSB
                }
                AddRing(CMD_FB_READ);

                ezDSP_uState = STAT_INIT;
                TX_FF_START;
            }
            else if(ezDSP_ucRx == CMD_DATA2B) {
                ezDSP_ulAddr++;	// auto increment

                ezDSP_uState = STAT_DATA2B;
                ezDSP_uDataRdCnt = 0;
            }
            else if(ezDSP_ucRx == CMD_DATA4B) {
                ezDSP_ulAddr += 2;	// auto increment

                ezDSP_uState = STAT_DATA4B;
                ezDSP_uDataRdCnt = 0;
            }
            else if(ezDSP_ucRx == CMD_DATA8B) {
                ezDSP_ulAddr += 4;	// auto increment

                ezDSP_uState = STAT_DATA8B;
                ezDSP_uDataRdCnt = 0;
            }
#if C2834x
            else if(ezDSP_ucRx == CMD_FLASH) {
                ezDSP_uState = STAT_FLASH;
                ezDSP_uFlashRdCnt = 0;
            }
#endif
        }
        else if(ezDSP_uState == STAT_ADDR) {
            ezDSP_uAddrRdCnt++;
            if(ezDSP_uAddrRdCnt == 1) {
                ezDSP_ulAddr = ezDSP_ucRx; 			// MSB
                ezDSP_ulAddr <<= 16; 				// MSB
            }
            else if(ezDSP_uAddrRdCnt == 2)
                ezDSP_ulAddr |= (ezDSP_ucRx << 8);

            else if(ezDSP_uAddrRdCnt == 3)
                ezDSP_ulAddr |= ezDSP_ucRx;			// LSB

            else if(ezDSP_uAddrRdCnt == 4) {
                if(ezDSP_ucRx == CMD_READ2B) {
                    ezDSP_uData = *(unsigned int*)ezDSP_ulAddr;

                    AddRing(ezDSP_uData >> 8);	    // MSB
                    AddRing(ezDSP_uData);		    // LSB
                    AddRing(CMD_FB_READ);

                    ezDSP_uState = STAT_INIT;
                    TX_FF_START;
                }
                else if(ezDSP_ucRx == CMD_READ4B) {	// modified at v9.4
                    ezDSP_ulData = *(unsigned long *)ezDSP_ulAddr;
                    AddRing(ezDSP_ulData >> 24);  // MSB
                    AddRing(ezDSP_ulData >> 16);
                    AddRing(ezDSP_ulData >> 8);
                    AddRing(ezDSP_ulData);        // LSB
                    AddRing(CMD_FB_READ);
                    ezDSP_uState = STAT_INIT;
                    TX_FF_START;
                }
                else if(ezDSP_ucRx == CMD_READ8B) {  // modified at v9.4
                    ezDSP_ullData = *(unsigned long long *)ezDSP_ulAddr;
                    AddRing(ezDSP_ullData >> (8*7));   // MSB
                    AddRing(ezDSP_ullData >> (8*6));
                    AddRing(ezDSP_ullData >> (8*5));
                    AddRing(ezDSP_ullData >> (8*4));
                    AddRing(ezDSP_ullData >> (8*3));
                    AddRing(ezDSP_ullData >> (8*2));
                    AddRing(ezDSP_ullData >> (8*1));
                    AddRing(ezDSP_ullData);            // LSB
                    AddRing(CMD_FB_READ);
                    ezDSP_uState = STAT_INIT;
                    TX_FF_START;
                }
                else if(ezDSP_ucRx == CMD_READ16B) {
                    for(uIndex = 0; uIndex < 8; uIndex++) {
                        // Since this is not for variable, addresss is increased sequentially
                        ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + uIndex);
                        AddRing(ezDSP_uData >> 8);		// MSB
                        AddRing(ezDSP_uData);			// LSB
                    }
                    AddRing(CMD_FB_READ);

                    ezDSP_uState = STAT_INIT;
                    TX_FF_START;
                }
                else if(ezDSP_ucRx == CMD_DATA2B) {
                    ezDSP_uState = STAT_DATA2B;
                    ezDSP_uDataRdCnt = 0;
                }
                else if(ezDSP_ucRx == CMD_DATA4B) {
                    ezDSP_uState = STAT_DATA4B;
                    ezDSP_uDataRdCnt = 0;
                }
                else if(ezDSP_ucRx == CMD_DATA8B) {
                    ezDSP_uState = STAT_DATA8B;
                    ezDSP_uDataRdCnt = 0;
                }
                else ezDSP_uState = STAT_INIT;
            }
            else
                ezDSP_uState = STAT_INIT;
        }
        else if(ezDSP_uState == STAT_DATA2B) {
            ezDSP_uDataRdCnt++;
            if(ezDSP_uDataRdCnt == 1)
                ezDSP_uData = ezDSP_ucRx << 8; 		// MSB
            else if(ezDSP_uDataRdCnt == 2)
                ezDSP_uData |= ezDSP_ucRx; 			// LSB
            else if(ezDSP_uDataRdCnt == 3)
                ezDSP_uChksum = ezDSP_ucRx << 8;	// MSB
            else if(ezDSP_uDataRdCnt == 4)
                ezDSP_uChksum |= ezDSP_ucRx;		// LSB
            else if(ezDSP_uDataRdCnt == 5) {
                if(ezDSP_ucRx == CMD_WRITE) {
                    if(ezDSP_uChksum == ((ezDSP_ulAddr + ezDSP_uData) & 0xFFFF)) {
                        *(unsigned int*)ezDSP_ulAddr = ezDSP_uData;
                        AddRing(CMD_FB_WRITE_OK);
                        ezDSP_uState = STAT_INIT;
                    }
                    else {
                        AddRing(CMD_FB_WRITE_NG);
                        ezDSP_uState = STAT_INIT;
                    }
                    TX_FF_START;
                }
                else
                    ezDSP_uState = STAT_INIT;
            }
            else
                ezDSP_uState = STAT_INIT;
        }
        else if(ezDSP_uState == STAT_DATA4B) {
            ezDSP_uDataRdCnt++;
            if(ezDSP_uDataRdCnt == 1) {
                ezDSP_ulData = ezDSP_ucRx; 		// MSB
                ezDSP_ulData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 2) {
                ezDSP_ulData |= ezDSP_ucRx;
                ezDSP_ulData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 3) {
                ezDSP_ulData |= ezDSP_ucRx;
                ezDSP_ulData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 4) {
                ezDSP_ulData |= ezDSP_ucRx;
            }
            else if(ezDSP_uDataRdCnt == 5)
                ezDSP_uChksum = ezDSP_ucRx << 8;	// MSB
            else if(ezDSP_uDataRdCnt == 6)
                ezDSP_uChksum |= ezDSP_ucRx;		// LSB
            else if(ezDSP_uDataRdCnt == 7) {
                if(ezDSP_ucRx == CMD_WRITE) {
                    if(ezDSP_uChksum == ((ezDSP_ulAddr + ezDSP_ulData) & 0xFFFF)) {
                        *(unsigned long*)ezDSP_ulAddr = ezDSP_ulData;
                        AddRing(CMD_FB_WRITE_OK);
                        ezDSP_uState = STAT_INIT;
                    }
                    else {
                        AddRing(CMD_FB_WRITE_NG);
                        ezDSP_uState = STAT_INIT;
                    }
                    TX_FF_START;
                }
                else
                    ezDSP_uState = STAT_INIT;
            }
            else
                ezDSP_uState = STAT_INIT;
        }
        else if(ezDSP_uState == STAT_DATA8B) {
            ezDSP_uDataRdCnt++;
            if(ezDSP_uDataRdCnt == 1) {
                ezDSP_ullData = ezDSP_ucRx; 		// MSB
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 2) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 3) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 4) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 5) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 6) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 7) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 8) {
                ezDSP_ullData |= ezDSP_ucRx;
            }
            else if(ezDSP_uDataRdCnt == 9)
                ezDSP_uChksum = ezDSP_ucRx << 8;	// MSB
            else if(ezDSP_uDataRdCnt == 10)
                ezDSP_uChksum |= ezDSP_ucRx;		// LSB
            else if(ezDSP_uDataRdCnt == 11) {
                if(ezDSP_ucRx == CMD_WRITE) {
                    if(ezDSP_uChksum == ((ezDSP_ulAddr + ezDSP_ullData) & 0xFFFF)) {
                        *(unsigned long long*)ezDSP_ulAddr = ezDSP_ullData;
                        AddRing(CMD_FB_WRITE_OK);
                        ezDSP_uState = STAT_INIT;
                    }
                    else {
                        AddRing(CMD_FB_WRITE_NG);
                        ezDSP_uState = STAT_INIT;
                    }
                    TX_FF_START;
                }
                else
                    ezDSP_uState = STAT_INIT;
            }
            else
                ezDSP_uState = STAT_INIT;
        }
		
#if C2834x
		else if(ezDSP_uState == STAT_FLASH) {
			ezDSP_uFlashRdCnt++;

			if(ezDSP_uFlashRdCnt == 1) {
				if(ezDSP_ucRx == CMD_FLASH_WRITE_READ || ezDSP_ucRx == CMD_FLASH_WRITE_ONLY)	{
					ezDSP_uFlashRdCnt = 9;
					ezDSP_ucFlashCmd = ezDSP_ucRx;
				}
				else {
					ezDSP_uFlashErrorCount++;
					ezDSP_uFlashErrorFlag |= SPIFLASHERR_WRONGCMD;
					ezDSP_uState = STAT_INIT;
				}
			}
			// write
			else if(ezDSP_uFlashRdCnt == 10) {
				ezDSP_uFlashDataCount = ezDSP_ucRx;
				if(ezDSP_uFlashDataCount == 0) ezDSP_uFlashDataCount = 1;	// if oveflow, it could be zero and makes infinite loop in below line.
			}
			else if(ezDSP_uFlashRdCnt >= 11 && ezDSP_uFlashRdCnt <= (10 + ezDSP_uFlashDataCount)){
				// Chip enable low at first
				if(ezDSP_uFlashRdCnt == 11) {
					GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
				
					// RX FF clear
					while(SpiaRegs.SPIFFRX.bit.RXFFST) {
						if(SpiaRegs.SPIFFRX.bit.RXFFST > 1) {	// only 1 byte is allowable
							ezDSP_uFlashErrorFlag |= SPIFLASHERR_SPIRXOVER;
							ezDSP_uFlashErrorCount++;
						}
						ezDSP_ucFlashRead =	SpiaRegs.SPIRXBUF;
					}
				}

				//FIFO check if full
				if(SpiaRegs.SPIFFTX.bit.TXFFST == 16) {
					ezDSP_uFlashErrorFlag |= SPIFLASHERR_SPITXFULL;
					ezDSP_uFlashErrorCount++;
				}
			
				// write
				SpiaRegs.SPITXBUF = (ezDSP_ucRx << 8);

				// read, and then send if necessary
				if(ezDSP_ucFlashCmd == CMD_FLASH_WRITE_READ) {
					// forward SPIRXBUF to SCI
					//if(SpiaRegs.SPIFFRX.bit.RXFFST) {	// invalidated in v8.52
					while(SpiaRegs.SPIFFRX.bit.RXFFST) {
						if(SpiaRegs.SPIFFRX.bit.RXFFST > 1) {
							ezDSP_uFlashErrorFlag |= SPIFLASHERR_SPIRXOVER;
							ezDSP_uFlashErrorCount++;						
						}

						if(ezDSP_uFlashRdCnt < 12) {
							ezDSP_uFlashErrorFlag |= SPIFLASHERR_SPIRXDELAY;
							ezDSP_uFlashErrorCount ++;
						}

						ezDSP_ucFlashRead = SpiaRegs.SPIRXBUF;
						AddRing(ezDSP_ucFlashRead);
					}
				}
				else if(ezDSP_ucFlashCmd == CMD_FLASH_WRITE_ONLY) {
					while(SpiaRegs.SPIFFRX.bit.RXFFST) ezDSP_ucFlashRead = SpiaRegs.SPIRXBUF;
				}
				else
					ezDSP_uFlashErrorCount++;
			
				// exit
				if(ezDSP_uFlashRdCnt == (10 + ezDSP_uFlashDataCount)) {
				    // Chip enable - high
					GpioDataRegs.GPASET.bit.GPIO19 = 1;
					ezDSP_uState = STAT_INIT;
				}
			}
		}		
#endif		
        else {
            ezDSP_uState = STAT_INIT;
        }
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag
	INT_NESTING_END;
}

#if C2834x
unsigned int ezDSP_uSPIFlashInitError = 0;
void easyDSP_SPI_Flashrom_Init(void)
{
    EALLOW;

    // clock enable
    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;   // SPI-A

    /* Enable internal pull-up for the selected pins */
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up on GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO19 (SPISTEA)

    /* Set qualification for selected pins to asynch only */
    // This will select asynch (no qualification) for the selected pins.
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)

    /* Configure SPI-A pins using GPIO regs*/
    // This specifies which of the possible GPIO pins will be SPI functional pins.
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA

    // IOPORT as output pin instead of SPISTE
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;

    // Chip enable - high
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    EDIS;

    // Initialize SPI FIFO registers
    SpiaRegs.SPIFFTX.all = 0xE040;
    SpiaRegs.SPIFFRX.all = 0x605f;                // mine
    SpiaRegs.SPIFFCT.all = 0x0;                   // no delay necessary since SCI comm makes it automatically

    // Init SPI
    SpiaRegs.SPICCR.all = 0x0047;                // Reset on, clk polarity=1, 8-bit char bits
    SpiaRegs.SPICTL.all = 0x0006;                // Enable master mode, normal phase,
                                                // enable talk, and SPI int disabled.

    // SPI Baudrate = 1M around
    // No need for fast SPI baudrate by considering slow byte-to-byte SCI comm (max,230400bps)
    SpiaRegs.SPIBRR = (int)(((float)LSP_CLK/(1000000L) - 1) + 0.5);
    //SpiaRegs.SPIBRR =0x0;                     // quick
    //SpiaRegs.SPIBRR =0x007F;                  // slow

    SpiaRegs.SPICCR.all = 0x00C7;                // Relinquish SPI from Reset
    SpiaRegs.SPIPRI.bit.FREE = 1;               // Set so breakpoints don't disturb xmission

    // confirm status
    if(SpiaRegs.SPIFFTX.bit.TXFFST != 0) ezDSP_uSPIFlashInitError++;
    if(SpiaRegs.SPIFFRX.bit.RXFFST != 0) ezDSP_uSPIFlashInitError++;
}
#endif

#if !IS_FIFO_16_LEVEL

#define BUFFER_COUNT 20
char ezDSP_ringBuffer[BUFFER_COUNT];
int ezDSP_nStartPos=0, ezDSP_nEndPos=0;
int ezDSP_nMaxBuffSize = 0;

void AddRing(unsigned char y) {
    int nBuffSize;

    ezDSP_ringBuffer[ezDSP_nEndPos] = y;
    ezDSP_nEndPos++;
    if(ezDSP_nEndPos == BUFFER_COUNT) ezDSP_nEndPos = 0;

    // check if buffer is full
    if(ezDSP_nStartPos == ezDSP_nEndPos) {
        ezDSP_nStartPos++;
        if(ezDSP_nStartPos == BUFFER_COUNT) ezDSP_nStartPos = 0;
    }
    // check maximum buff size
    else {
        if(ezDSP_nEndPos >= ezDSP_nStartPos) 
            nBuffSize =  ezDSP_nEndPos - ezDSP_nStartPos;
        else 
            nBuffSize = BUFFER_COUNT + ezDSP_nEndPos - ezDSP_nStartPos;

        if(nBuffSize > ezDSP_nMaxBuffSize) 
            ezDSP_nMaxBuffSize = nBuffSize;
    }
}

char ExtractRing(void) {
    char rv;

    rv = ezDSP_ringBuffer[ezDSP_nStartPos++];
    if(ezDSP_nStartPos == BUFFER_COUNT) ezDSP_nStartPos = 0;

    return rv;
}

inline int IsRingEmpty(void)
{
    return ezDSP_nEndPos == ezDSP_nStartPos;
}
#endif
