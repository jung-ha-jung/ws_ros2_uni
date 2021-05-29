#ifndef __MODBUS_MAP_H_
#define __MODBUS_MAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "dataType.h"

// ----- Function Code -----
#define		FC1			0x01
#define		FC2			0x02
#define		FC3			0x03
#define		FC4			0x04
#define		FC5			0x05
#define		FC6			0x06
#define		FC15		0x0F
#define		FC16		0x10
#define		FC24		0x18

// Exception Codes
#define		ILLEGAL_FUNCTION						0x01
#define		ILLEGAL_DATA_ADDRESS					0x02
#define		ILLEGAL_DATA_VALUE						0x03
#define		ILLEGAL_RESPONSE_LENGTH					0x04
#define		ACKNOWLEDGE								0x05
#define		SLAVE_DEVICE_BUSY						0x06
#define		NEGATIVE_ACKNOWLEDGE					0x07
#define		MEMORY_PARITY_ERROR						0x08
#define		GATEWAY_PATH_UNAVAILABLE				0x0A
#define		GATEWAY_TARGET_DEVICE_FAIL_TO_RESPOND	0x0B

// ====================================================================== //

#if 1//ndef _VRC
	#define	BE1	^1	// Big Endian for 1bit
	#define	BE2	^3	// Big Endian for 2bit
#else
	#define	BE1 	// Little Endian
	#define	BE2 	// Little Endian
#endif

extern	u8	ucaBitData[];
extern	u8	ucaInvBitData[];

extern	u16	usaBitData[];

extern	u16	usaInvBitData[];


#define	mChku8Bit(u8Array,Offset,BitNo)	(u8Array[(Offset)+(BitNo)/8] & ucaBitData[(BitNo) % 8])
#define	mSetu8Bit(u8Array,Offset,BitNo,SetReset)	( ( SetReset ) ? \
					(u8Array[(Offset)+(BitNo)/8] |= ucaBitData[(BitNo) % 8] ) \
				: 	(u8Array[(Offset)+(BitNo)/8] &= ucaInvBitData[(BitNo) % 8] ))
#define	mChkBit(RT,RNr)	(stRelayAdd[(RT)].RelayAdd[(RNr)/16].us \
										& usaBitData[(RNr) % 16])
#define	mSetBit(RT,RNr,SetReset)	(\
			( SetReset ) ? \
				(stRelayAdd[(RT)].RelayAdd[(RNr)/16].us \
					|= usaBitData[(RNr) % 16] ) \
			: 	(stRelayAdd[(RT)].RelayAdd[(RNr)/16].us \
					&= usaInvBitData[(RNr) % 16] ))

typedef union {
	s8	c[2];
	u8	uc[2];
	s16	s;
	u16	us;
	//s32	l;
	//u32	ul;
	//float	f;
}	csUnion;//}   cslfUnion;

typedef	struct {
	u16	usSize;
	csUnion	*RelayAdd;//csUnion	*RelayAdd;
} RelayAddType;

extern	const	RelayAddType stRelayAdd[];

typedef struct {
	u16	RelType;
	u16	StartAddr;
}	RelType;
extern	const	RelType	GroupRelType[][14];

//	I/Os for Max	//Map	//CAN_F	//DF	//(ROP)
#define	MAX_AI	64//16//32		//4		//16+4	//16+4
#define	MAX_XW	64//4//128		//1		//16+16	//32+16
#define	MAX_TC	64//16//32		//4		//16	//0
#define	MAX_RTD	64//4//32		//0(3)	//0		//10

#define	MAX_AO	64//8//32		//2		//8		//8
#define	MAX_YW	64//4//128		//1		//32+16	//32+8

#define	LE_XW	32 // idtXW[??]�� LineError ����

#define	MAX_DataMem		10000	//
#define	MAX_RopMem		2000	//
#define	MAX_SM1Mem		1000	//
#define	MAX_SysMem		1000
#define	MAX_BckUpMem	10000	//
#define	MAX_rBckUpMem	2000	//

extern	IoDataType	idtXW[];
extern	IoDataType	idtYW[];
extern	IoDataType	idtAI[];
extern	IoDataType	idtAO[];
extern	IoDataType	idtTC[];
extern	IoDataType	idtRTD[];

extern	IoDataType	DataMem[];
extern	IoDataType	RopMem[];
extern	IoDataType	SM1Mem[];
extern	IoDataType	SysMem[];

#define rBckUpMem_Addr		0x60001000
#define BckUpMem_Addr		(rBckUpMem_Addr + MAX_rBckUpMem*2)
#define BckUpDataMem_Addr	(BckUpMem_Addr + MAX_BckUpMem*2)

#define rBckUpMem			((IoDataType*)rBckUpMem_Addr)
#define BckUpMem			((IoDataType*)BckUpMem_Addr)
#define BckUpDataMem		((IoDataType*)BckUpDataMem_Addr)

s32	GetRelayInfo(u16 Group, u16 usAddress, u16 *uspStartAdd, u16 *uspEndAdd, u16 *uspRelayType );
s16 GetModbusData(u16 usAddress);
void SetModbusData(u16 usAddress, s16 sData);
IoDataType GetDataMem(u16 usAddr);
void SetDataMem(u16 usAddr, s16 sData);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* __MODBUS_MAP_H_ */
