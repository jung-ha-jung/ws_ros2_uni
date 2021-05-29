// ====================================================================== //
//				Modbus tcp master program
// ====================================================================== //

#ifndef __MODBUS_TCP_M_H
#define __MODBUS_TCP_M_H

#include "dataType.h"

#ifdef __cplusplus
extern "C" {
#endif

// ������ TCP master ���� -------------------------------------
#define	MAX_MODBUS_TCP_M	1 // �ִ� ��

typedef struct {
	u8 ucFunction;
	u16 usAddress;
	u16 usLength;
} ModbusTcpMQueryType;

typedef struct {
	u8 ucConnection;
	u8 ucaRemoteIp[4];
	u16 usRemotePort;
	u8 ucSlaveID;
	u16 usScanRate;
	u16 usTimeout;
	u8 ucRetry;
	ModbusTcpMQueryType Query[10];
	int state;
	int sockfd;
} ModbusTcpMCfgType;

extern ModbusTcpMCfgType g_staModbusTcpMCfg[MAX_MODBUS_TCP_M];

typedef struct {
	u16			TID;			// Transaction ID
	u16			PID;			// Protocol ID
	u16			Len;			// Length
	u8			UID;			// Unit ID
	u8			Func;			// Function Code
	u8			Data[300];		// Data // modified 300 --> 8,000 by Sunny
} ModbusPacketType;

typedef struct {
	int 					Rx_New_Data;				// Flag receive modbus-TCP packet completed.
	int 					Tx_Data_Rdy;				// Flag Ready Status to send response packet.
	ModbusPacketType		Mod_RX;						// Rx data buffer
	ModbusPacketType		Mod_TX;						// Tx data buffer
	u8	Send_Modbus_Packet[300];	// Send Mobus-TCP buffer // modified 300 --> 8,000 by Sunny
} ModbusTCPMBufType;

extern ModbusTCPMBufType g_stModbusTCPMBuf;
typedef int SOCKET;

// ====================================================================== //
//	Function Declaration
// ---------------------------------------------------------------------- //
void ModbusTcpMCfgInit();
int modbusTcpm_init(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg);
int modbusTcpm_send(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg);
int modbusTcpm_read(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg);
void modbusTcpm_proc(u16 task_ms);
void ModbusTCP_M_CfgGet(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg);
void ModbusTCP_M_CfgSet(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg);
void ModbusTCP_M_BufGet(u8 ucMaster);
void ModbusTCP_M_BufSet(u8 ucMaster);
s32 ModbusTCP_M_Connect(u8 ucMaster);
void ModbusTCP_M_SockInit(u8 ucMaster);
int ModbusTCP_M_IsScan(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg);
s32 ModbusTCP_M_TxData(SOCKET s, u8 ucMaster);
void ModbusTCP_M_RxData(u8* buf, u32 len, u8 ucMaster);
s32 Process_FC3_M_Req(u8 ucMaster);
s32 Process_FC3_M_Res();
s32 Process_FC16_M_Req(u8 ucMaster);
s32 Process_FC16_M_Res();
s32 Process_Modbus_TCP_M_Req(u8 ucMaster);
s32 Process_Modbus_TCP_M_Res();
void Modbus_Rx_M_Buffer(u8* TCP_Rx_data, u16 TCP_Rx_data_len);

#ifdef __cplusplus
}
#endif

#endif  // __MODBUS_TCP_M_H
