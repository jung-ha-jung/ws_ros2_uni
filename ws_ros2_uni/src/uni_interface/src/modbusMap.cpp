#include "uni_interface/modbusMap.h"


u8	ucaBitData[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
u8	ucaInvBitData[] = { (u8)~0x01, (u8)~0x02, (u8)~0x04, (u8)~0x08, (u8)~0x10, (u8)~0x20, (u8)~0x40, (u8)~0x80 };

u16	usaBitData[] = {
					0x0001, 0x0002, 0x0004, 0x0008,
					0x0010, 0x0020, 0x0040, 0x0080,
					0x0100, 0x0200, 0x0400, 0x0800,
					0x1000, 0x2000, 0x4000, 0x8000 };

u16	usaInvBitData[] = {
					(u16)(~0x0001), (u16)(~0x0002), (u16)(~0x0004), (u16)(~0x0008),
					(u16)(~0x0010), (u16)(~0x0020), (u16)(~0x0040), (u16)(~0x0080),
					(u16)(~0x0100), (u16)(~0x0200), (u16)(~0x0400), (u16)(~0x0800),
					(u16)(~0x1000), (u16)(~0x2000), (u16)(~0x4000), (u16)(~0x8000) };


/* Relay Type No. ----------------------------- */
enum {
	REY_DMb=0, 	REY_DM,
	REY_X,		REY_XW,
	REY_Y,		REY_YW,
	REY_AIb, 	REY_AI,
	REY_AOb, 	REY_AO,
	REY_TCb, 	REY_TC,
	REY_RTDb, 	REY_RTD,
	REY_RMb, 	REY_RM,
	REY_SM1b, 	REY_SM1,
	REY_BMb, 	REY_BM,
	REY_SMb, 	REY_SM,
	REY_rBMb, 	REY_rBM
};

const	RelayAddType stRelayAdd[] = {
	//		Bit									Word
	{ MAX_DataMem,	(csUnion *)DataMem },	{ MAX_DataMem,	(csUnion *)DataMem },
	{ MAX_XW*16,	(csUnion *)idtXW },	{ MAX_XW,	(csUnion *)idtXW },
	{ MAX_YW*16,	(csUnion *)idtYW },	{ MAX_YW,	(csUnion *)idtYW },
	{ MAX_AI,		(csUnion *)idtAI },	{ MAX_AI,	(csUnion *)idtAI },
	{ MAX_AO,		(csUnion *)idtAO },	{ MAX_AO,	(csUnion *)idtAO },
	{ MAX_TC,		(csUnion *)idtTC },	{ MAX_TC,	(csUnion *)idtTC },
	{ MAX_RTD,		(csUnion *)idtRTD },{ MAX_RTD,	(csUnion *)idtRTD },
	{ MAX_RopMem,	(csUnion *)RopMem },	{ MAX_RopMem,	(csUnion *)RopMem },
	{ MAX_SM1Mem,	(csUnion *)SM1Mem },	{ MAX_SM1Mem,	(csUnion *)SM1Mem },
	{ MAX_BckUpMem,	(csUnion *)BckUpMem },	{ MAX_BckUpMem,	(csUnion *)BckUpMem },
	{ MAX_SysMem,	(csUnion *)SysMem },	{ MAX_SysMem,	(csUnion *)SysMem },
	{ MAX_rBckUpMem,(csUnion *)rBckUpMem },	{ MAX_rBckUpMem,(csUnion *)rBckUpMem },
};


const	RelType	GroupRelType[][14]= {
	{	// 0: Group 0: CoilBit
		{ REY_X,		10000},
		{ REY_Y,		12000},
		{ 0,			0xffff},
	},
	{	// 1: Group 1: InDisc
		{ REY_X,		10000},
		{ 0,			0xffff},
	},
	{	// 2: Group 3: InReg
		{ REY_XW,		10000},
		{ REY_AI,		10400},
		{ REY_TC,		10800},
		{ REY_RTD,		11000},
		{ 0,			0xffff},
	},
	{	// 3: Group 4: HoldReg
		{ REY_DM,		0},
		{ REY_XW,		10000},
		{ REY_YW,		10200},
		{ REY_AI,		10400},
		{ REY_AO,		10600},
		{ REY_TC,		10800},
		{ REY_RTD,		11000},
		{ REY_RM,		15000},
		{ REY_SM1,		17000},
		{ REY_BM,		20000},
		{ REY_SM,		30000},
		{ REY_rBM,		40000},
		{ 0,			0xffff}
	}
};


//#pragma SET_DATA_SECTION("SDRAM_DATA")
// Codesys���� %M(memory)�� ���� modbus map ��ü�� access�ϱ� ���ؼ� linear memory�� �䱸�Ǵµ� �̸� ���ؼ� ������ ������
// �ϱ��� ������ �����ؾ� �ϸ� �ʿ�� ������å�ӿ��� ���� ���!
IoDataType	DataMem[MAX_DataMem];		// [    0 ~  9999]

IoDataType	idtXW[MAX_XW];				// [10000 ~ 10063]
IoDataType	r_idtXW[200-MAX_XW];		// [10064 ~ 10199] -> for linear memory
IoDataType	idtYW[MAX_YW];				// [10200 ~ 10263]
IoDataType	r_idtYW[200-MAX_YW];		// [10264 ~ 10299] -> for linear memory
IoDataType	idtAI[MAX_AI];				// [10400 ~ 10463]
IoDataType	r_idtAI[200-MAX_AI];		// [10464 ~ 10599] -> for linear memory
IoDataType	idtAO[MAX_AO];				// [10600 ~ 10663]
IoDataType	r_idtAO[200-MAX_AO];		// [10664 ~ 10799] -> for linear memory
IoDataType	idtTC[MAX_TC];				// [10800 ~ 10863]
IoDataType	r_idtTC[200-MAX_TC];		// [10864 ~ 10999] -> for linear memory
IoDataType	idtRTD[MAX_RTD];			// [11000 ~ 11063]
IoDataType	r_idtRTD[4000-MAX_RTD];		// [11064 ~ 14999] -> for linear memory

IoDataType	RopMem[MAX_RopMem];			// [15000 ~ 16999]
IoDataType	SM1Mem[MAX_SM1Mem];			// [17000 ~ 17999]
IoDataType	rSM1Mem[13000-MAX_SM1Mem];	// [18000 ~ 29999] -> for linear memory
IoDataType	SysMem[MAX_SysMem];			// [30000 ~ 30999]
//#pragma SET_DATA_SECTION()


/* --------------------------------------------------------
���� :
���ϰ� :

-------------------------------------------------------- */
s32	GetRelayInfo(u16 Group, u16 usAddress, u16 *uspStartAdd, u16 *uspEndAdd, u16 *uspRelayType )
{
	u8	i;
	for( i=0;1; i++ ) {
		if( usAddress >= GroupRelType[Group][i].StartAddr ) {
			*uspRelayType = GroupRelType[Group][i].RelType;
			*uspStartAdd = GroupRelType[Group][i].StartAddr;
			continue;
		}
		break;
	}
	*uspEndAdd = *uspStartAdd + stRelayAdd[*uspRelayType].usSize;
	//prints( "G=%i,Add=%i,StAd=%i,EdAd=%,RT=%i\r\n", Group, usAddress, *uspStartAdd, *uspEndAdd, *uspRelayType );
	return 0;
}


/* --------------------------------------------------------
���� : usAddress(0 Based)
���ϰ� : ������
������ �����͸� ����
-------------------------------------------------------- */
s16 GetModbusData(u16 usAddress)
{
	u16	usQuantity=1, usStartAdd, usEndAdd, usRelayType;

	GetRelayInfo( 4-1, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	if( usAddress < usStartAdd || usAddress+usQuantity > usEndAdd )
		return 0;
	usAddress -= usStartAdd;
	return stRelayAdd[usRelayType].RelayAdd[usAddress].s;
}


/* --------------------------------------------------------
���� : usAddress(0 Based)
���ϰ� :
������ �����͸� ����
-------------------------------------------------------- */
void SetModbusData(u16 usAddress, s16 sData)
{
	u16	usQuantity=1, usStartAdd, usEndAdd, usRelayType;

	GetRelayInfo( 4-1, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	if( usAddress < usStartAdd || usAddress+usQuantity > usEndAdd )
		return;
	usAddress -= usStartAdd;
	stRelayAdd[usRelayType].RelayAdd[usAddress].s = sData;
}


/* --------------------------------------------------------
���� : usAddr(0 Based)
���ϰ� :
������ �޸𸮰��� ����
-------------------------------------------------------- */
IoDataType GetDataMem(u16 usAddr)
{
	IoDataType Tmp;

	if(usAddr >= MAX_DataMem) {
		//prints("GetDataMem(%d); error!\r\n", usAddr);
		Tmp.u16_data = 0;
	}
	else {
		Tmp = DataMem[usAddr];
	}

	return Tmp;
}


/* --------------------------------------------------------
���� : usAddr(0 Based)
���ϰ� :
������ �޸𸮰��� ����
-------------------------------------------------------- */
void SetDataMem(u16 usAddr, s16 sData)
{
	if(usAddr >= MAX_DataMem)
		; //prints("SetDataMem(%d); error!\r\n", usAddr);
	else
		DataMem[usAddr].s16_data = sData;
}

