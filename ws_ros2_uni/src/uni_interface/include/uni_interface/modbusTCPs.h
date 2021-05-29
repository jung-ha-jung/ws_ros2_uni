#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H


#ifdef __cplusplus
extern "C" {
#endif

#include <netinet/in.h>
#include "dataType.h"
#include "modbusMap.h"


// ====================================================================== //
//	Define
// ---------------------------------------------------------------------- //
#define	MAX_MODBUS_TCP	1 // �ִ� ��


// ====================================================================== //
//	Global Structure
// ---------------------------------------------------------------------- //
typedef struct {
	u8 ucConnection;
	u16 usPort;
	u8 ucSlaveID;
	int state;
	int sock_listen;
	int sock_child;
	struct sockaddr_in client_sock_addr_ip;
	socklen_t client_sock_addr_ip_size;
} ModbusTcpCfgType;

extern ModbusTcpCfgType g_staModbusTcpCfg[];

struct s_Modbus_Packet {
	u16			TID;			// Transaction ID
	u16			PID;			// Protocol ID
	u16			Len;			// Length
	u8			UID;			// Unit ID
	u8			Func;			// Function Code
	u8			Data[300];		// Data // modified 300 --> 8,000 by Sunny
};

struct s_Modbus_TCP {
	int 						Rx_New_Data;				// Flag receive modbus-TCP packet completed.
	int 						Tx_Data_Rdy;				// Flag Ready Status to send response packet.
	struct	s_Modbus_Packet		Mod_RX;						// Rx data buffer
	struct	s_Modbus_Packet		Mod_TX;						// Tx data buffer
	u8				Send_Modbus_Packet[300];	// Send Mobus-TCP buffer // modified 300 --> 8,000 by Sunny
};
extern struct s_Modbus_TCP g_staModbus_TCP_buffer[];


// ====================================================================== //
//	Function Declaration
// ---------------------------------------------------------------------- //
void ModbusTcpCfgInit();
int modbusTcps_init(ModbusTcpCfgType *stpModbusTcpCfg);
int modbusTcps_accept(ModbusTcpCfgType *stpModbusTcpCfg);
int modbusTcps_receive(u8 ucSlave, ModbusTcpCfgType *stpModbusTcpCfg);
void modbusTcps_proc(void);
void Modbus_Buffer_Init (void);
void Modbus_Rx_Buffer (u8 ucSlave, u8* TCP_Rx_data, u16 TCP_Rx_data_len);
void Process_Modbus_TCP (u8 ucSlave);
void Process_FC1 (u8 ucSlave);
void Process_FC2 (u8 ucSlave);
void Process_FC3 (u8 ucSlave);
void Process_FC4 (u8 ucSlave);
void Process_FC5 (u8 ucSlave);
void Process_FC6 (u8 ucSlave);
void Process_FC15 (u8 ucSlave);
void Process_FC16 (u8 ucSlave);
void Process_Exception (u8 ucSlave, u8 Exception_code);


/***************************************************************************
*                             TRACING
***************************************************************************/
#ifndef  TRACE_LEVEL_OFF
#define  TRACE_LEVEL_OFF                                 0
#endif
#ifndef  TRACE_LEVEL_INFO
#define  TRACE_LEVEL_INFO                                1
#endif
#ifndef  TRACE_LEVEL_DBG
#define  TRACE_LEVEL_DBG                                 2
#endif
#ifndef  MODs_TRACE_LEVEL
#define  MODs_TRACE_LEVEL                       TRACE_LEVEL_OFF
#endif
#ifndef  MODs_TRACE
#define  MODs_TRACE                             printf
#endif

#if    ((defined(MODs_TRACE))       && \
        (defined(MODs_TRACE_LEVEL)) && \
        (MODs_TRACE_LEVEL >= TRACE_LEVEL_INFO) )

    #if  (MODs_TRACE_LEVEL >= TRACE_LEVEL_DBG)
        #define  MODs_TRACE_DBG(msg)     MODs_TRACE  msg
    #else
        #define  MODs_TRACE_DBG(msg)
    #endif

    #define  MODs_TRACE_INFO(msg)        MODs_TRACE  msg

#else
    #define  MODs_TRACE_DBG(msg)
    #define  MODs_TRACE_INFO(msg)
#endif


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of MODBUS_TCP_H definition

// ====================================================================== //
// End of file.
// ====================================================================== //

