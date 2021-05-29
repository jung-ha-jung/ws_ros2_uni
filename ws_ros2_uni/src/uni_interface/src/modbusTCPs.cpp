// ====================================================================== //
//	Includes
// ====================================================================== //
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <netdb.h>
#include "uni_interface/modbusMap.h"
#include "uni_interface/modbusTCPs.h"


ModbusTcpCfgType g_staModbusTcpCfg[MAX_MODBUS_TCP];
struct s_Modbus_TCP g_staModbus_TCP_buffer[MAX_MODBUS_TCP];
#define  RX_BUF_SIZE       300
#define  BACKLOG           5 /* how many pending connections queue will hold */

/* --------------------------------------------------------
param [in,out] :
return :
modbus config initialize
-------------------------------------------------------- */
void ModbusTcpCfgInit()
{
	g_staModbusTcpCfg[0].ucConnection = 1;
	g_staModbusTcpCfg[0].usPort = 5000;
	g_staModbusTcpCfg[0].ucSlaveID = 1;
	g_staModbusTcpCfg[0].state = 0;
	g_staModbusTcpCfg[0].sock_listen = 0;
	g_staModbusTcpCfg[0].sock_child = 0;
}


/* --------------------------------------------------------
param [in,out] :
return :
modbus slave initialize
-------------------------------------------------------- */
int modbusTcps_init(ModbusTcpCfgType *stpModbusTcpCfg)
{
	struct sockaddr_in server_sock_addr_ip;
	int sock_listen;
	int ret;

	sock_listen = socket(AF_INET, SOCK_STREAM, 0); /* ----------------- OPEN IPV4 SOCKET ----------------- */
	if (sock_listen == -1) {
		printf("Server : Can't open stream socket\n");
    return -1;
	}
  fcntl(sock_listen, F_SETFL, O_NONBLOCK);

	memset(&server_sock_addr_ip, 0x00, sizeof(server_sock_addr_ip)); // initialize
	server_sock_addr_ip.sin_family = AF_INET;
  server_sock_addr_ip.sin_addr.s_addr = htonl(INADDR_ANY);
  server_sock_addr_ip.sin_port = htons(stpModbusTcpCfg->usPort);

	ret = bind(sock_listen, (struct sockaddr *)&server_sock_addr_ip, sizeof(server_sock_addr_ip)); /* ------------------- BIND SOCKET -------------------- */
	if (ret < 0) {
		printf("Server %d(%d) : Can't bind local address.\n", sock_listen, stpModbusTcpCfg->usPort);
		close(sock_listen);
		return -1;
	}
	//printf("Server : bind local address.\n");

	ret = listen(sock_listen, BACKLOG); /* ------------------ LISTEN SOCKET ------------------- */
	if (ret < 0) {
		printf("Server : Can't listening connect.\n");
		close(sock_listen);
    return -1;
	}
	//printf("Server : listening connect.\n");

	stpModbusTcpCfg->sock_listen = sock_listen;

	return 1;
}


/* --------------------------------------------------------
factor :
return :
modbus slave service
-------------------------------------------------------- */
int modbusTcps_accept(ModbusTcpCfgType *stpModbusTcpCfg)
{
	stpModbusTcpCfg->client_sock_addr_ip_size = sizeof(stpModbusTcpCfg->client_sock_addr_ip);
	stpModbusTcpCfg->sock_child = accept(stpModbusTcpCfg->sock_listen, (struct sockaddr *)&stpModbusTcpCfg->client_sock_addr_ip, &stpModbusTcpCfg->client_sock_addr_ip_size); /* ---------- ACCEPT NEW INCOMING CONNECTION ---------- */
	//printf("%d = accept()\n", (int)sock_child);

	if (stpModbusTcpCfg->sock_child < 0) { // not accepted
		// printf("Server : accept failed.\n");
		return -1;
	}

	fcntl(stpModbusTcpCfg->sock_child, F_SETFL, O_NONBLOCK);
	//printf("Server : accept success.\n");

	return 1;
}


/* --------------------------------------------------------
factor :
return :
modbus slave service
-------------------------------------------------------- */
int modbusTcps_read(u8 ucSlave, ModbusTcpCfgType *stpModbusTcpCfg)
{
	ssize_t rx_size;
	ssize_t tx_size;
	ssize_t tx_rem;
	char rx_buf[RX_BUF_SIZE];
	char temp[20];
	u8 *p_buf;
	int ret;

	inet_ntop(AF_INET, &stpModbusTcpCfg->client_sock_addr_ip.sin_addr.s_addr, temp, sizeof(temp));
	//printf("Server : %s client connected.\n", temp);

	rx_size = recv(stpModbusTcpCfg->sock_child, rx_buf, RX_BUF_SIZE, 0); /* ----- WAIT UNTIL RECEIVING DATA FROM A CLIENT ------ */
	//printf("%d = recv()\n", (int)rx_size);
	switch (rx_size) {
	case -1: // not received
		// printf("_____ recv() error\n");
		return 0;
	case 0: // close
		printf("Close\n");
		ret = close(stpModbusTcpCfg->sock_child); /* ---------------- CLOSE CHILD SOCKET ---------------- */
		if (ret == -1) {
			close(stpModbusTcpCfg->sock_listen); /* This function should be reached only when a fatal ...*/
			return -2;
		}
		return -1;
	default: // received
		//printf("recv ok : %d\n", (int)rx_size);
		Modbus_Rx_Buffer(ucSlave, (u8*)rx_buf, rx_size); // Get Recv data

		Process_Modbus_TCP(ucSlave); // Modbus Func Process

		if(g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy == 1)
		{
			p_buf = &g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[0];
			tx_rem = (6+g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len);
			do {
			   tx_size = sendto(stpModbusTcpCfg->sock_child, (void *)p_buf, tx_rem, 0, (struct sockaddr *)&stpModbusTcpCfg->client_sock_addr_ip, stpModbusTcpCfg->client_sock_addr_ip_size); /* ----- TRANSMIT THE DATA RECEIVED TO THE CLIENT ----- */
			   tx_rem -= tx_size;
			   p_buf = (u8 *)(p_buf + tx_size);
			} while (tx_rem > 0);
		}
		break;
	}

	return 1;
}


/* --------------------------------------------------------
factor :
return :
modbus slave service
-------------------------------------------------------- */
void modbusTcps_proc(void)
{
	enum modbusTcps_state { STATE_MOD_TCPS_INIT,
		STATE_MOD_TCPS_ACCEPT, STATE_MOD_TCPS_READ, STATE_MOD_TCPS_ERROR };
	ModbusTcpCfgType *stpModbusTcpCfg;
	u8 ucSlave;
	int ret;

	for(ucSlave=0; ucSlave<MAX_MODBUS_TCP; ucSlave++) {
		stpModbusTcpCfg = &g_staModbusTcpCfg[ucSlave];
		if(stpModbusTcpCfg->ucConnection < 1)
			continue;

		switch (stpModbusTcpCfg->state) {
		case STATE_MOD_TCPS_INIT :
			ret = modbusTcps_init(stpModbusTcpCfg);
			if(ret > 0)
				stpModbusTcpCfg->state = STATE_MOD_TCPS_ACCEPT;
			else
				stpModbusTcpCfg->state = STATE_MOD_TCPS_ERROR;
			break;
		case STATE_MOD_TCPS_ACCEPT:
			ret = modbusTcps_accept(stpModbusTcpCfg);
			if(ret > 0)
				stpModbusTcpCfg->state = STATE_MOD_TCPS_READ;
			break;
		case STATE_MOD_TCPS_READ:
			ret = modbusTcps_read(ucSlave, stpModbusTcpCfg);
			switch( ret ) {
			case -1:
				stpModbusTcpCfg->state = STATE_MOD_TCPS_ACCEPT;
				break;
			case -2:
				stpModbusTcpCfg->state = STATE_MOD_TCPS_ERROR;
				break;
			}
			break;
		}
	}
}


// ====================================================================== //
//	Function: Modbus_Buffer_Init
//		- Initialize Modbus Buffer
// ---------------------------------------------------------------------- //
void Modbus_Buffer_Init (void)
{
	u8 ucSlave;

	for(ucSlave=0; ucSlave<MAX_MODBUS_TCP; ucSlave++)
	{
		g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
	}
}


// ====================================================================== //
//	Function: Modbus_Rx_Buffer
//		- Save the Modbus-TCP Packet.
// ---------------------------------------------------------------------- //
void Modbus_Rx_Buffer (u8 ucSlave, u8* TCP_Rx_data, u16 TCP_Rx_data_len)
{
	int i = 0;

	u16	temp16 = 0;
	u8	temp8_1 = 0;
	u8	temp8_2 = 0;

	if ( TCP_Rx_data_len > 0 )
	{
		// Transaction ID
		temp8_1 = *TCP_Rx_data;
		temp8_2 = *(TCP_Rx_data+1);
		temp16 = (((u16)(temp8_1))<<8)+(u16)(temp8_2);

		g_staModbus_TCP_buffer[ucSlave].Mod_RX.TID = temp16;


		// Protocol ID
		temp8_1 = *(TCP_Rx_data+2);
		temp8_2 = *(TCP_Rx_data+3);
		temp16 = (((u16)(temp8_1))<<8)+(u16)(temp8_2);

		g_staModbus_TCP_buffer[ucSlave].Mod_RX.PID = temp16;


		// Length
		temp8_1 = *(TCP_Rx_data+4);
		temp8_2 = *(TCP_Rx_data+5);
		temp16 = (((u16)(temp8_1))<<8)+(u16)(temp8_2);

		g_staModbus_TCP_buffer[ucSlave].Mod_RX.Len = temp16;


		// Uint ID
		g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID = *(TCP_Rx_data+6);


		// Function Code
		g_staModbus_TCP_buffer[ucSlave].Mod_RX.Func = *(TCP_Rx_data+7);//ucaInFrame[1]


		// Data
		for ( i=0; i < (g_staModbus_TCP_buffer[ucSlave].Mod_RX.Len-2); i++)
		{
			g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[i] = *(TCP_Rx_data+(8+i));//ucaInFrame[2~]
		}

		// Set flag Rx new data.
		g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 1;
	}
	else
	{
		// Modbus-TCP Packet is net received.
		g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
	}
}


// ====================================================================== //
//	Function: Process_FC1
//		- Processing Modbus-TCP, Function code FC-01: Read Coils
//		- Group 0, Command - FC1
// ---------------------------------------------------------------------- //
void Process_FC1 (u8 ucSlave)
{
	int i =0;

	u16		usQuantity;
	u16		usAddress;
	u16		usStartAdd;
	u16		usEndAdd;
	u16		usRelayType;
	u8		ucByteCnt;

	// Set Unit ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID;

	// Set Function Code - FC-3(Read Mutiple Registers)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func = FC1;

	// Get Reference number
	usAddress = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1]));

	// Get Word Count
	usQuantity = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3]));

	if( usQuantity < 1 || usQuantity > 2039 )
	{
		// usQuantity is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_VALUE);
		return;
	}
	GetRelayInfo( 0, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	// Check the Word Count is in valid range. (Data Length should be less than 125.)
	if( (usAddress < usStartAdd) || (usAddress+usQuantity > usEndAdd) )
	{
		// usAddress is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_ADDRESS);
		return;
	}

	usAddress -= usStartAdd;

	// Set Byte Count of Response
	ucByteCnt = (u8)((usQuantity-1)/8+1);
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = ucByteCnt;

	for( i=0; i<ucByteCnt; i++ )
		g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[1+i] = 0;
	for( i=0; i<usQuantity; i++ )
		mSetu8Bit( g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data, 1, i, mChkBit(usRelayType, usAddress+i) );

	// Set Length (Unit ID + Func Code + Byte Count + Data[Byte_Count])
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len = ucByteCnt+3;

	// Tx Ready Flag Set
	g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy = 1;

	// Clear flag Rx new data.
	g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
}


// ====================================================================== //
//	Function: Process_FC2
//		- Processing Modbus-TCP, Function code FC-02: Read Input Discrete
//		- Group 1, Command - FC2
// ---------------------------------------------------------------------- //
void Process_FC2 (u8 ucSlave)
{
	int i =0;

	u16		usQuantity;
	u16		usAddress;
	u16		usStartAdd;
	u16		usEndAdd;
	u16		usRelayType;
	u8		ucByteCnt;

	// Set Unit ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID;

	// Set Function Code - FC-3(Read Mutiple Registers)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func = FC2;

	// Get Reference number
	usAddress = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1]));

	// Get Word Count
	usQuantity = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3]));

	if( usQuantity < 1 || usQuantity > 2039 )
	{
		// usQuantity is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_VALUE);
		return;
	}
	GetRelayInfo( 1, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	// Check the Word Count is in valid range. (Data Length should be less than 125.)
	if( (usAddress < usStartAdd) || (usAddress+usQuantity > usEndAdd) )
	{
		// usAddress is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_ADDRESS);
		return;
	}

	usAddress -= usStartAdd;

	// Set Byte Count of Response
	ucByteCnt = (u8)((usQuantity-1)/8+1);
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = ucByteCnt;

	for( i=0; i<ucByteCnt; i++ )
		g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[1+i] = 0;
	for( i=0; i<usQuantity; i++ )
		mSetu8Bit( g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data, 1, i, mChkBit(usRelayType, usAddress+i) );

	// Set Length (Unit ID + Func Code + Byte Count + Data[Byte_Count])
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len = ucByteCnt+3;

	// Tx Ready Flag Set
	g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy = 1;

	// Clear flag Rx new data.
	g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
}


// ====================================================================== //
//	Function: Process_FC3
//		- Processing Modbus-TCP, Function code FC3-Read Multiple Registers.
//		- Group 4, Command - FC3
// ---------------------------------------------------------------------- //
void Process_FC3 (u8 ucSlave)
{
	int i =0;

	u16		usQuantity;
	u16		usAddress;
	u16		usStartAdd;
	u16		usEndAdd;
	u16		usRelayType;

	// Set Unit ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID;

	// Set Function Code - FC-3(Read Mutiple Registers)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func = FC3;

	// Get Reference number
	usAddress = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1]));

	// Get Word Count
	usQuantity = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3]));

	if( usQuantity < 1 || usQuantity > 127 )
	{
		// usQuantity is NOT Valid value -> Occur Exception.
		//prints( "Here is FC3(Q=%i\r\n",usQuantity);
		Process_Exception(ucSlave, ILLEGAL_DATA_VALUE);
		return;
	}
	GetRelayInfo( 4-1, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	// Check the Word Count is in valid range. (Data Length should be less than 125.)
	if( (usAddress < usStartAdd) || (usAddress+usQuantity > usEndAdd) )
	{
		// usAddress is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_ADDRESS);
		return;
	}

	usAddress -= usStartAdd;

	// Set Byte Count of Response (Byte Count = 2 x Word Count)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = (u8)(usQuantity*2);

	for( i=0; i<usQuantity*2; i++ )
		g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[1+i] = stRelayAdd[usRelayType].RelayAdd[usAddress+i/2].uc[i%2 BE1];

	// Set Length (Unit ID + Func Code + Byte Count + Data[Byte_Count])
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len = usQuantity*2+3;

	// Tx Ready Flag Set
	g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy = 1;

	// Clear flag Rx new data.
	g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
}


// ====================================================================== //
//	Function: Process_FC4
//		- Processing Modbus-TCP, Function code FC-04: Read Input Register
//		- Group 3, Command - FC4
// ---------------------------------------------------------------------- //
void Process_FC4 (u8 ucSlave)
{
	int i =0;

	u16		usQuantity;
	u16		usAddress;
	u16		usStartAdd;
	u16		usEndAdd;
	u16		usRelayType;

	// Set Unit ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID;

	// Set Function Code - FC-3(Read Mutiple Registers)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func = FC4;

	// Get Reference number
	usAddress = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1]));

	// Get Word Count
	usQuantity = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3]));

	if( usQuantity < 1 || usQuantity > 127 )
	{
		// usQuantity is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_VALUE);
		return;
	}
	GetRelayInfo( 3-1, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	// Check the Word Count is in valid range. (Data Length should be less than 125.)
	if( (usAddress < usStartAdd) || (usAddress+usQuantity > usEndAdd) )
	{
		// usAddress is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_ADDRESS);
		return;
	}

	usAddress -= usStartAdd;

	// Set Byte Count of Response (Byte Count = 2 x Word Count)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = (u8)(usQuantity*2);

	for( i=0; i<usQuantity*2; i++ )
		//g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[1+i] = stRelayAdd[usRelayType].RelayAdd[usAddress+i/2].uc[(usAddress+i)%2 BE1];
		g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[1+i] = stRelayAdd[usRelayType].RelayAdd[usAddress+i/2].uc[i%2 BE1];

	// Set Length (Unit ID + Func Code + Byte Count + Data[Byte_Count])
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len = usQuantity*2+3;

	// Tx Ready Flag Set
	g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy = 1;

	// Clear flag Rx new data.
	g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
}


// ====================================================================== //
//	Function: Process_FC5
//		- Processing Modbus-TCP, Function code FC-05: Write Single Coil
//		- Class 1 Command - FC5
// ---------------------------------------------------------------------- //
void Process_FC5 (u8 ucSlave)
{
	u16		usData;
	//u16		usQuantity;
	u16		usAddress;
	u16		usStartAdd;
	u16		usEndAdd;
	u16		usRelayType;

	// Set Unit ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID;

	// Set Function Code - FC-6(Write Single Registers)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func = FC5;

	// Get Reference number
	usAddress = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1]));
	usData = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3]));

	if( usData != 0 && usData != 0xff00 )
	{
		// usQuantity is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_VALUE);
		return;
	}
	GetRelayInfo( 0, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	if( usAddress < usStartAdd || usAddress >= usEndAdd )
	{
		// usAddress is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_ADDRESS);
		return;
	}
	usAddress -= usStartAdd;

	mSetBit(usRelayType,usAddress,usData);
	usData = (mChkBit(usRelayType,usAddress))?0xff00:0x0000;

	// Send Reference number
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0];
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[1] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1];

	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[2] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2];
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[3] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3];

	// Set Length (Unit ID(1) + Func Code(1) + Reference number(2) + Register Value(2))
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len = 6;

	// Tx Ready Flag Set
	g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy = 1;

	// Clear flag Rx new data.
	g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
}


// ====================================================================== //
//	Function: Process_FC6
//		- Processing Modbus-TCP, Function code FC6-Write Single Register.
//		- Class 1 Command - FC6
// ---------------------------------------------------------------------- //
void Process_FC6 (u8 ucSlave)
{
	u16		usData;
	//u16		usQuantity;
	u16		usAddress;
	u16		usStartAdd;
	u16		usEndAdd;
	u16		usRelayType;

	// Set Unit ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID;

	// Set Function Code - FC-6(Write Single Registers)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func = FC6;

	// Get Reference number
	usAddress = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1]));
	usData = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3]));

	GetRelayInfo( 4-1, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	if( usAddress < usStartAdd || usAddress >= usEndAdd )
	{
		// usAddress is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_ADDRESS);
		return;
	}
	usAddress -= usStartAdd;

	stRelayAdd[usRelayType].RelayAdd[usAddress].us = usData;

	// Send Reference number
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0];
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[1] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1];

	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[2] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2];
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[3] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3];

	// Set Length (Unit ID(1) + Func Code(1) + Reference number(2) + Register Value(2))
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len = 6;

	// Tx Ready Flag Set
	g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy = 1;

	// Clear flag Rx new data.
	g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
}


// ====================================================================== //
//	Function: Process_FC15
//		- Processing Modbus-TCP, Function code FC-015: Write Multiple Coils
//		- Group 0, Command - FC15
// ---------------------------------------------------------------------- //
void Process_FC15 (u8 ucSlave)
{
	int i =0;

	u16		usQuantity;
	u16		usAddress;
	u16		usStartAdd;
	u16		usEndAdd;
	u16		usRelayType;

	// Set Unit ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID;

	// Set Function Code - FC-3(Read Mutiple Registers)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func = FC15;

	// Get Reference number
	usAddress = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1]));

	// Get Word Count
	usQuantity = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3]));

	if( usQuantity < 1 || usQuantity > 2039
		|| usQuantity > g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[4]*8 )
	{
		// usQuantity is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_VALUE);
		return;
	}
	GetRelayInfo( 0, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	// Check the Word Count is in valid range. (Data Length should be less than 125.)
	if( (usAddress < usStartAdd) || (usAddress+usQuantity > usEndAdd) )
	{
		// usAddress is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_ADDRESS);
		return;
	}

	usAddress -= usStartAdd;

	// Set Byte Count of Response (Byte Count = 2 x Word Count)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = (u8)(usQuantity*2);

	for( i=0; i<usQuantity; i++ ) {
		mSetBit(usRelayType,usAddress+i,mChku8Bit(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data,5, i ) );
	}

	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0];
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[1] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1];

	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[2] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2];
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[3] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3];

	// Set Length (Unit ID + Func Code + Byte Count + Data[Byte_Count])
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len = 6;

	// Tx Ready Flag Set
	g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy = 1;

	// Clear flag Rx new data.
	g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
}


// ====================================================================== //
//	Function: Process_FC16
//		- Processing Modbus-TCP, Function code FC-16: Write Multiple Registers
//		- Group 4, Command - FC16
// ---------------------------------------------------------------------- //

void Process_FC16 (u8 ucSlave)
{
	int i =0;

	u16		usQuantity;
	u16		usAddress;
	u16		usStartAdd;
	u16		usEndAdd;
	u16		usRelayType;

	// Set Unit ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID;

	// Set Function Code - FC-3(Read Mutiple Registers)
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func = FC16;

	// Get Reference number
	usAddress = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1]));

	// Get Word Count
	usQuantity = ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2]<<8)) + ((u16)(g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3]));

	if( usQuantity < 1 || usQuantity > 127
		|| usQuantity*2 != g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[4] )
	{
		// usQuantity is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_VALUE);
		return;
	}
	GetRelayInfo( 4-1, usAddress, &usStartAdd, &usEndAdd, &usRelayType );
	// Check the Word Count is in valid range. (Data Length should be less than 125.)
	if( (usAddress < usStartAdd) || (usAddress+usQuantity > usEndAdd) )
	{
		// usAddress is NOT Valid value -> Occur Exception.
		Process_Exception(ucSlave, ILLEGAL_DATA_ADDRESS);
		return;
	}

	usAddress -= usStartAdd;

	for( i=0; i<usQuantity; i++ )
		stRelayAdd[usRelayType].RelayAdd[(usAddress+i)].us = (u16)((((u16)g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[5+i*2])<<8) + g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[6+i*2]);

	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[0];
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[1] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[1];

	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[2] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[2];
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[3] = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Data[3];


	// Set Length (Unit ID + Func Code + Byte Count + Data[Byte_Count])
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len = 6;

	// Tx Ready Flag Set
	g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy = 1;

	// Clear flag Rx new data.
	g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
}


// ====================================================================== //
//	Function: Process_Exception
//		- Make Exceoption Packet.
// ---------------------------------------------------------------------- //
void Process_Exception (u8 ucSlave, u8 Exception_code)
{
	// Transaction ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.TID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.TID;

	// Protocol ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.PID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.PID;

	// Set Unit ID
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.UID;

	// Set Exception Function Code - FC3
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func = g_staModbus_TCP_buffer[ucSlave].Mod_RX.Func+0x80;

	// Set Exception Code
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[0] = Exception_code;

	// Set Length
	g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len = 3;

	// Tx Ready Flag Set
	g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy = 1;

	// Clear flag Rx new data.
	g_staModbus_TCP_buffer[ucSlave].Rx_New_Data = 0;
}


// ====================================================================== //
//	Function: Process_Modbus_TCP
//		- Processing Rx Modbus-TCP Packet to send respose packet.
// ---------------------------------------------------------------------- //
void Process_Modbus_TCP (u8 ucSlave)
{
	int i = 0;

	if ( g_staModbus_TCP_buffer[ucSlave].Rx_New_Data == 1 )
	{
		// Transaction ID
		g_staModbus_TCP_buffer[ucSlave].Mod_TX.TID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.TID;

		// Protocol ID
		g_staModbus_TCP_buffer[ucSlave].Mod_TX.PID = g_staModbus_TCP_buffer[ucSlave].Mod_RX.PID;

		// Process Function Code
		switch (g_staModbus_TCP_buffer[ucSlave].Mod_RX.Func)
		{
			// ----- Group 0 ----- //

			// FC-01: Read Coils
			case FC1:
				Process_FC1(ucSlave);
				break;

			// FC-05: Write Single Coil
			case FC5:
				Process_FC5(ucSlave);
				break;

			// FC-015: Write Multiple Coils
			case FC15:
				Process_FC15(ucSlave);
				break;


			// ----- Group 1 ----- //

			// FC-02: Read Input Discrete
			case FC2:
				Process_FC2(ucSlave);
				break;


			// ----- Group 3 ----- //

			//FC-04: Read Input Register
			case FC4:
				Process_FC4(ucSlave);
				break;

			// ----- Group 4 ----- //

			// FC-03: Read Multiple Registers
			case FC3:
				Process_FC3(ucSlave);
				break;

			// FC-06: Write Single Register
			case FC6:
				Process_FC6(ucSlave);
				break;

			// FC-16: Write Multiple Registers
			case FC16:
				Process_FC16(ucSlave);
				break;

			// Undefined Function. Send Exception packet.
			default:
				Process_Exception(ucSlave, ILLEGAL_FUNCTION);
				break;
		}
	}


	// Make Send Modbus Packet
	if ( g_staModbus_TCP_buffer[ucSlave].Tx_Data_Rdy == 1)
	{
		// Transaction ID
		g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[0] = (u8)(g_staModbus_TCP_buffer[ucSlave].Mod_TX.TID >> 8);
		g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[1] = (u8)(g_staModbus_TCP_buffer[ucSlave].Mod_TX.TID & 0x00FF);

		// Protocol ID
		g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[2] = (u8)(g_staModbus_TCP_buffer[ucSlave].Mod_TX.PID >> 8);
		g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[3] = (u8)(g_staModbus_TCP_buffer[ucSlave].Mod_TX.PID & 0x00FF);

		// Length
		g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[4] = (u8)(g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len >> 8);
		g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[5] = (u8)(g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len & 0x00FF);


		// Unit ID
		g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[6] = g_staModbus_TCP_buffer[ucSlave].Mod_TX.UID;

		// Function Code
		g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[7] = g_staModbus_TCP_buffer[ucSlave].Mod_TX.Func;

		// Data
		for (i=0; i < (g_staModbus_TCP_buffer[ucSlave].Mod_TX.Len-2); i++)
		{
			g_staModbus_TCP_buffer[ucSlave].Send_Modbus_Packet[8+i] = g_staModbus_TCP_buffer[ucSlave].Mod_TX.Data[i];
		}
	}
}

