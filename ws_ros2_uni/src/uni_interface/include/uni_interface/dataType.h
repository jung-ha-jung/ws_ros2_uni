/*
 * DataType.h
 *
 *  Created on: 2014. 12. 6.
 *      Author: Sunny Kang
 */

#ifndef DATATYPE_H_
#define DATATYPE_H_

#ifdef __cplusplus
extern "C" {
#endif

//Open---------------------------------------------------------------------------
#define CANTESTSM2 1

#define PASS		0
#define FAIL		-1

#define ENABLE		1
#define DISABLE		0

#define ON			1
#define OFF			0

typedef char				s8;
typedef short				s16;
typedef int					s32;
typedef long long			s64;
typedef unsigned char		u8;
typedef unsigned short		u16;
typedef unsigned int		u32;
typedef unsigned long long	u64;

typedef unsigned long		Uint32;
typedef unsigned short		Uint16;

typedef	union {
	u8	u8_data[2];
	u16	u16_data;
	s16	s16_data;
} IoDataType;
//===============================================================================

typedef int					sint;
typedef int					BOOL;
typedef unsigned char		UCHAR;
typedef unsigned char   	U_08;
typedef   signed char   	S_08;
typedef unsigned short  	U_16;
typedef   signed short  	S_16;
typedef unsigned long   	U_32;
typedef   signed long   	S_32;

//	Ring buffer for CAN message
typedef struct {	// RingBuff[32]AC indexing
	u16	bIn:5;		u16	:0;
	u16	bOut:5;		u16	:0;
} b5PtrType;
typedef struct {	// RingBuff[64]AC indexing
	u16	bIn:6;		u16	:0;
	u16	bOut:6;		u16	:0;
} b6PtrType;
typedef struct {	// RingBuff[128]AC indexing
	u16	bIn:7;		u16	:0;
	u16	bOut:7;		u16	:0;
} b7PtrType;
typedef struct {	// RingBuff[256]AC indexing
	u16	bIn:8;		u16	:0;
	u16	bOut:8;		u16	:0;
} b8PtrType;
typedef struct {	// RingBuff[512]AC indexing
	u16	bIn:9;		u16	:0;
	u16	bOut:9;		u16	:0;
} b9PtrType;
typedef struct {	// RingBuff[1024]AC indexing
	u16	bIn:10;		u16	:0;
	u16	bOut:10;	u16	:0;
} b10PtrType;
typedef struct {	// RingBuff[2048]AC indexing
	u16	bIn:11;		u16	:0;
	u16	bOut:11;	u16	:0;
} b11PtrType;
typedef struct {	// RingBuff[4096]AC indexing
	u16	bIn:12;		u16	:0;
	u16	bOut:12;	u16	:0;
} b12PtrType;
typedef struct {	// RingBuff[8192]AC indexing
	u16	bIn:13;		u16	:0;
	u16	bOut:13;	u16	:0;
} b13PtrType;


#ifdef __cplusplus
}
#endif

#endif /* DATATYPE_H_ */
