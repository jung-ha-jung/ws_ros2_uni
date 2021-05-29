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
#include "uni_interface/dataType.h"
#include "uni_interface/modbusMap.h"
#include "uni_interface/modbusTCPs.h"
#include "uni_interface/modbusTCPm.h"

ModbusTcpMCfgType g_staModbusTcpMCfg[MAX_MODBUS_TCP_M];
ModbusTCPMBufType g_stModbusTCPMBuf; // Master ������ŭ �־�������� �̴� ����..
u8 g_ucaQuery[MAX_MODBUS_TCP_M];
u16 g_usaElapsedMs[MAX_MODBUS_TCP_M] = {0,};
s32 g_iaResponseTimeOut[MAX_MODBUS_TCP_M];
u8 g_ucaRetry[MAX_MODBUS_TCP_M];
#define  RX_BUF_SIZE       300


/* --------------------------------------------------------
param [in,out] :
return :
modbus config initialize
-------------------------------------------------------- */
void ModbusTcpMCfgInit()
{
	g_staModbusTcpMCfg[0].ucConnection = 1;
	g_staModbusTcpMCfg[0].ucaRemoteIp[0] = 192;
	g_staModbusTcpMCfg[0].ucaRemoteIp[1] = 168;
	g_staModbusTcpMCfg[0].ucaRemoteIp[2] = 0;
	g_staModbusTcpMCfg[0].ucaRemoteIp[3] = 253;
	g_staModbusTcpMCfg[0].usRemotePort = 5000;
	g_staModbusTcpMCfg[0].ucSlaveID = 1;
	g_staModbusTcpMCfg[0].usScanRate = 10;
	g_staModbusTcpMCfg[0].usTimeout = 5000;
	g_staModbusTcpMCfg[0].ucRetry = 2;
	g_staModbusTcpMCfg[0].state = 0;
	g_staModbusTcpMCfg[0].sockfd = 0;
	g_staModbusTcpMCfg[0].Query[0].ucFunction = 3;
	g_staModbusTcpMCfg[0].Query[0].usAddress = 9000;
	g_staModbusTcpMCfg[0].Query[0].usLength = 50;
	g_staModbusTcpMCfg[0].Query[1].ucFunction = 16;
	g_staModbusTcpMCfg[0].Query[1].usAddress = 9500;
	g_staModbusTcpMCfg[0].Query[1].usLength = 50;
}


int connect_nonb(int sockfd, const struct sockaddr *saptr, int salen, int nsec);
int tcp_connect_timeo(const char *hostname, const char *service, int nsec)
{
     struct addrinfo hints, *res, *ressave;
     int  sock,n;

     bzero(&hints, sizeof(struct addrinfo));
     hints.ai_family = AF_UNSPEC;
     hints.ai_socktype = SOCK_STREAM;

     if( (n = getaddrinfo(hostname, service, &hints, &res)) != 0)
         return -1;
     ressave = res;
     do {
//         struct  sockaddr_in *ts;
         sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
         if(sock < 0)
             continue;

//         ts = (struct sockaddr_in *) res->ai_addr;

         if(connect_nonb(sock, (struct sockaddr *)res->ai_addr, res->ai_addrlen,nsec) == 0)
             break;
         close(sock);
     } while( (res=res->ai_next) !=NULL);
     if( res == NULL)
         return -1;
     freeaddrinfo(ressave);
     return sock;
}


int connect_nonb(int sockfd, const struct sockaddr *saptr, int salen, int nsec)
{
    int             flags, n, error;
    socklen_t       len;
    fd_set          rset, wset;
    struct timeval  tval;

    flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    error = 0;
    if ( (n = connect(sockfd, (struct sockaddr *) saptr, salen)) < 0)
        if (errno != EINPROGRESS)
            return(-1);

    /* Do whatever we want while the connect is taking place. */

    if (n == 0)
        goto done;  /* connect completed immediately */

    FD_ZERO(&rset);
    FD_SET(sockfd, &rset);
    wset = rset;
    tval.tv_sec = nsec;
    tval.tv_usec = 0;

    if ( (n = select(sockfd+1, &rset, &wset, NULL, nsec ? &tval : NULL)) == 0) {
        close(sockfd);      /* timeout */
        errno = ETIMEDOUT;
        return(-1);
    }

    if (FD_ISSET(sockfd, &rset) || FD_ISSET(sockfd, &wset)) {
        len = sizeof(error);
        if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len) < 0)
            return(-1);         /* Solaris pending error */
    }
    else {
        printf("select error: sockfd not set");
    }

done:
    fcntl(sockfd, F_SETFL, flags);  /* restore file status flags */
    if (error) {
        close(sockfd);      /* just in case */
        errno = error;
        return(-1);
    }
    return(0);
}


/* --------------------------------------------------------
���� :
���ϰ� :
modbus master service
-------------------------------------------------------- */
int modbusTcpm_init(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg)
{
	struct sockaddr_in server_sock_addr;
	struct hostent *he;
	char temp[20];
	int ret;

	sprintf(temp, "%d.%d.%d.%d", stpModbusTcpMCfg->ucaRemoteIp[0], stpModbusTcpMCfg->ucaRemoteIp[1], stpModbusTcpMCfg->ucaRemoteIp[2], stpModbusTcpMCfg->ucaRemoteIp[3]);

#if 0
	stpModbusTcpMCfg->sockfd = tcp_connect_timeo(temp, temp, 500000);
	if (stpModbusTcpMCfg->sockfd == -1) {
		printf("Server : Can't open stream socket\n");
		return -1;
	}
#else
	if ((he = gethostbyname(temp)) == NULL) {  /* get the host info */
		printf("gethostbyname() error.\n");
		return -1;
	}

	stpModbusTcpMCfg->sockfd = socket(AF_INET, SOCK_STREAM, 0); /* ----------------- OPEN IPV4 SOCKET ----------------- */
	if (stpModbusTcpMCfg->sockfd == -1) {
		printf("Server : Can't open stream socket\n");
		return -1;
	}
//	fcntl(stpModbusTcpMCfg->sockfd, F_SETFL, O_NONBLOCK);

	memset(&server_sock_addr, 0x00, sizeof(server_sock_addr)); // initialize
	server_sock_addr.sin_family = AF_INET;      /* host byte order */
	server_sock_addr.sin_port = htons(stpModbusTcpMCfg->usRemotePort);    /* short, network byte order */
	server_sock_addr.sin_addr = *((struct in_addr *)he->h_addr);

	printf("1-1..\n");
  /* ------------------ CONNECT SOCKET ------------------ */
  ret = connect(stpModbusTcpMCfg->sockfd, (struct sockaddr *)&server_sock_addr, sizeof(struct sockaddr));
	printf("connect..(%d)\n", ret);
	if (ret == -1) {
		printf("Server : Can't connect.\n");
		close(stpModbusTcpMCfg->sockfd);
    return -1;
	}
	printf("2..\n");
#endif
	ModbusTCP_M_SockInit(ucMaster);

	return 1;
}


/* --------------------------------------------------------
���� :
���ϰ� :
modbus master service
-------------------------------------------------------- */
int modbusTcpm_send(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg)
{
	if(ModbusTCP_M_IsScan(ucMaster, stpModbusTcpMCfg) == 0)
		return -1;

	ModbusTCP_M_TxData(stpModbusTcpMCfg->sockfd, ucMaster);

    return 1;
}


/* --------------------------------------------------------
���� :
���ϰ� :
modbus master service
-------------------------------------------------------- */
int modbusTcpm_read(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg)
{
	char rx_buf[RX_BUF_SIZE];
	ssize_t rx_size;

	rx_size = recv(stpModbusTcpMCfg->sockfd, rx_buf, RX_BUF_SIZE, 0); /* ----- WAIT UNTIL RECEIVING DATA FROM A CLIENT ------ */
	switch (rx_size) {
	case -1: // not received
		printf("MODm : RX_Q_EMPTY\r\n");
		return 0;
	case 0: // close
		printf("MODm NetSock_RxDataFrom() : line #%u.\n", (unsigned int)__LINE__);
		//close(stpModbusTcpMCfg->sockfd); /* ------------------- CLOSE SOCKET ------------------- */
		return -1;
	default: // received
//	printf("received\n");
		rx_buf[rx_size] = '\0';
		ModbusTCP_M_RxData((u8 *)rx_buf, rx_size, ucMaster);
		break;
	}

	return 1;
}


/* --------------------------------------------------------
���� :
���ϰ� :
modbus master service
-------------------------------------------------------- */
void modbusTcpm_proc(u16 task_ms)
{
	enum modbusTcpm_state { STATE_MOD_TCPM_INIT, STATE_MOD_TCPM_SEND, STATE_MOD_TCPM_READ, STATE_MOD_TCPM_ERROR };
	ModbusTcpMCfgType *stpModbusTcpMCfg;
	u8 ucMaster;
	int ret;

	for(ucMaster=0; ucMaster<MAX_MODBUS_TCP_M; ucMaster++) {
		stpModbusTcpMCfg = &g_staModbusTcpMCfg[ucMaster];
		if(stpModbusTcpMCfg->ucConnection < 1)
			continue;
		g_usaElapsedMs[ucMaster] += task_ms;

		switch (stpModbusTcpMCfg->state) {
		case STATE_MOD_TCPM_INIT :
			ret = modbusTcpm_init(ucMaster, stpModbusTcpMCfg);
			if(ret > 0)
				stpModbusTcpMCfg->state = STATE_MOD_TCPM_SEND;
			break;
		case STATE_MOD_TCPM_SEND:
			ret = modbusTcpm_send(ucMaster, stpModbusTcpMCfg);
			if(ret > 0)
				stpModbusTcpMCfg->state = STATE_MOD_TCPM_READ;
			break;
		case STATE_MOD_TCPM_READ:
			ret = modbusTcpm_read(ucMaster, stpModbusTcpMCfg);
			switch( ret ) {
			case -1:
				stpModbusTcpMCfg->state = STATE_MOD_TCPM_INIT;
				break;
			case 1:
				stpModbusTcpMCfg->state = STATE_MOD_TCPM_SEND;
				break;
			}
			break;
		}
	}
}


/* --------------------------------------------------------
���� : ucMaster=�����͹�ȣ(0 Based)
���ϰ� :
Modbus TCP ������ ����
-------------------------------------------------------- */
void ModbusTCP_M_CfgGet(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg)
{
	*stpModbusTcpMCfg = g_staModbusTcpMCfg[ucMaster];
}


/* --------------------------------------------------------
���� : ucMaster=�����͹�ȣ(0 Based)
���ϰ� :
Modbus TCP ����
-------------------------------------------------------- */
void ModbusTCP_M_CfgSet(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg)
{
	g_staModbusTcpMCfg[ucMaster] = *stpModbusTcpMCfg;
}


/* --------------------------------------------------------
���� :
���ϰ� : 0=�̿���, 1=����
Modbus TCP ������¸� ����
-------------------------------------------------------- */
s32 ModbusTCP_M_Connect(u8 ucMaster)
{
	if(ucMaster >= MAX_MODBUS_TCP_M)
		return 0;
	return g_staModbusTcpMCfg[ucMaster].ucConnection;
}


/* --------------------------------------------------------
���� :
���ϰ� :
�������� Query ��ȣ�� ����
-------------------------------------------------------- */
u8 GetQuery(u8 ucMaster)
{
	return g_ucaQuery[ucMaster];
}


/* --------------------------------------------------------
���� :
���ϰ� :
�������� Query ��ȣ�� ����
-------------------------------------------------------- */
void SetQuery(u8 ucMaster, u8 ucQuery)
{
	if(g_staModbusTcpMCfg[ucMaster].Query[ucQuery].ucFunction == 0) // ������ Query�� �ʰ�������
		ucQuery = 0;

	g_ucaQuery[ucMaster] = ucQuery;
}


/* --------------------------------------------------------
���� :
���ϰ� :
Modbus tcp master sock init ó��
-------------------------------------------------------- */
void ModbusTCP_M_SockInit(u8 ucMaster)
{
	SetQuery(ucMaster, 0); // ������ Ŭ����
	g_stModbusTCPMBuf.Tx_Data_Rdy = 0;
}


/* --------------------------------------------------------
���� :
���ϰ� :
Modbus tcp master scan ó��
-------------------------------------------------------- */
int ModbusTCP_M_IsScan(u8 ucMaster, ModbusTcpMCfgType *stpModbusTcpMCfg)
{
	if(g_usaElapsedMs[ucMaster] < stpModbusTcpMCfg->usScanRate)
		return 0;

	g_usaElapsedMs[ucMaster] = 0; // Ŭ����

	return 1;
}


/* --------------------------------------------------------
���� :
���ϰ� :
Modbus tcp master TX ó��
-------------------------------------------------------- */
s32 ModbusTCP_M_TxData(SOCKET s, u8 ucMaster)
{
	ModbusTcpMCfgType stModbusTcpMCfg;
	s32 iRet;

	ModbusTCP_M_CfgGet(ucMaster, &stModbusTcpMCfg);

	if(g_stModbusTCPMBuf.Tx_Data_Rdy == 0)
	{
		iRet = Process_Modbus_TCP_M_Req(ucMaster); // Modbus Master Request Process
		if(iRet > 0)
		{
			iRet = send(s, (u8*)&g_stModbusTCPMBuf.Send_Modbus_Packet, 6+g_stModbusTCPMBuf.Mod_TX.Len, 0); // Send Modbus Request.
			if(iRet <= 0) // Send �����̸�
			{
				printf("send <= 0\r\n");
				return iRet;
			}
			g_stModbusTCPMBuf.Tx_Data_Rdy = 1; // Request �Ϸ�
		}
	}
	else
	{
		g_iaResponseTimeOut[ucMaster] += stModbusTcpMCfg.usScanRate; // 10ms�� ���
		if(g_iaResponseTimeOut[ucMaster] >= stModbusTcpMCfg.usTimeout) // ��� �����̸�
		{
			g_iaResponseTimeOut[ucMaster] = 0; // timeout Ŭ����
			if(++g_ucaRetry[ucMaster] > stModbusTcpMCfg.ucRetry) // Retry ȸ���� �ʰ��ϸ�
			{
				// ErrorOutput(WARN, WARN_NET_COMM_ERR, GetQuery(ucMaster));
				g_ucaRetry[ucMaster] = 0; // Retry Ŭ����
				// SetQuery(ucMaster, (GetQuery(ucMaster)+1)); // ���� ������ �� -> Heart beat�� ���� Query�� �����̶�� ������ �Ǿ� ����.
			}
			g_stModbusTCPMBuf.Tx_Data_Rdy = 0; // Request
		}
	}

	return 1;
}


/* --------------------------------------------------------
���� :
���ϰ� :
Modbus tcp master RX ó��
-------------------------------------------------------- */
void ModbusTCP_M_RxData(u8* buf, u32 len, u8 ucMaster)
{
	s32 iRet;

	if(len >= 300)
	{
		printf("len >= 300\r\n");
		return;
	}

	Modbus_Rx_M_Buffer(buf, len); // Get Recv data

	iRet = Process_Modbus_TCP_M_Res(); // Modbus Func Process
	if(iRet < 1)
	{
		printf("Process_Modbus_TCP_M_Res() < 1\r\n");
		return;
	}

	SetQuery(ucMaster, (GetQuery(ucMaster)+1)); // ���� ������ ��
	g_stModbusTCPMBuf.Tx_Data_Rdy = 0; // Request
	g_iaResponseTimeOut[ucMaster] = 0; // timeout Ŭ����
	g_ucaRetry[ucMaster] = 0; // Retry Ŭ����
	// ErrorClear(WARN, WARN_NET_COMM_ERR, GetQuery(ucMaster));
}


/* --------------------------------------------------------
���� :
���ϰ� : <0=����
Function code FC3-Read Multiple Registers. ��û
-------------------------------------------------------- */
s32 Process_FC3_M_Req(u8 ucMaster)
{
	ModbusTcpMCfgType stModbusTcpMCfg;
	u16	usAddress, usQuantity;
	u8 ucQuery = GetQuery(ucMaster);

	ModbusTCP_M_CfgGet(ucMaster, &stModbusTcpMCfg);

	usAddress = stModbusTcpMCfg.Query[ucQuery].usAddress; // Start addr.
	usQuantity = stModbusTcpMCfg.Query[ucQuery].usLength; // Length
	if(usQuantity < 1 || usQuantity > 127)
		return -1;

	g_stModbusTCPMBuf.Mod_TX.Data[0] = (u8)(usAddress >> 8);
	g_stModbusTCPMBuf.Mod_TX.Data[1] = (u8)(usAddress & 0xFF);
	g_stModbusTCPMBuf.Mod_TX.Data[2] = (u8)(usQuantity >> 8);
	g_stModbusTCPMBuf.Mod_TX.Data[3] = (u8)(usQuantity & 0xFF);

	g_stModbusTCPMBuf.Mod_TX.Len = 6; // Set Length (Unit ID + Func Code + Start addr(2) + Quantity(2))
	g_stModbusTCPMBuf.Rx_New_Data = 0; // Clear flag Rx new data.

//	prints("FC3 usAddress=%d, usQuantity=%d\r\n", usAddress, usQuantity);

	return 1;
}


/* --------------------------------------------------------
���� :
���ϰ� :
Function code FC3-Read Multiple Registers. ����
-------------------------------------------------------- */
s32 Process_FC3_M_Res()
{
	u16	i, usAddress, usQuantity;
	s16 sData;

	usAddress = ((u16)(g_stModbusTCPMBuf.Mod_TX.Data[0]<<8)) + ((u16)(g_stModbusTCPMBuf.Mod_TX.Data[1])); // Get Reference number
	usQuantity = ((u16)(g_stModbusTCPMBuf.Mod_TX.Data[2]<<8)) + ((u16)(g_stModbusTCPMBuf.Mod_TX.Data[3])); // Get Word Count
	if((usQuantity*2) != g_stModbusTCPMBuf.Mod_RX.Data[0])
	{
		printf("(usQuantity*2) != g_stModbusTCPMBuf.Mod_RX.Data[0]\r\n");
		return -1;
	}

	for(i=0; i<usQuantity; i++)
	{
		sData = (s16)((((u16)g_stModbusTCPMBuf.Mod_RX.Data[1+(i*2)])<<8) + g_stModbusTCPMBuf.Mod_RX.Data[1+(i*2)+1]);
		SetDataMem(usAddress+i, sData);
	}

	g_stModbusTCPMBuf.Rx_New_Data = 0; // Clear flag Rx new data.

	return 1;
}


/* --------------------------------------------------------
���� :
���ϰ� :
Function code FC-16: Write Multiple Registers ��û
-------------------------------------------------------- */
s32 Process_FC16_M_Req(u8 ucMaster)
{
	ModbusTcpMCfgType stModbusTcpMCfg;
	u16	i, usAddress, usQuantity;
	u8 ucQuery = GetQuery(ucMaster);

	ModbusTCP_M_CfgGet(ucMaster, &stModbusTcpMCfg);

	usAddress = stModbusTcpMCfg.Query[ucQuery].usAddress; // Start address
	usQuantity = stModbusTcpMCfg.Query[ucQuery].usLength; // Length
	if( usQuantity < 1 || usQuantity > 127 )
		return -1;

	g_stModbusTCPMBuf.Mod_TX.Data[0] = (u8)(usAddress >> 8); // Address High
	g_stModbusTCPMBuf.Mod_TX.Data[1] = (u8)(usAddress & 0xFF); // Address Low
	g_stModbusTCPMBuf.Mod_TX.Data[2] = (u8)(usQuantity >> 8); // Quantity High
	g_stModbusTCPMBuf.Mod_TX.Data[3] = (u8)(usQuantity & 0xFF); // Quantity Low
	g_stModbusTCPMBuf.Mod_TX.Data[4] = (u8)(usQuantity*2); // Set Byte Count of Response (Byte Count = 2 x Word Count)

	for(i=0; i<usQuantity; i++)
	{
		g_stModbusTCPMBuf.Mod_TX.Data[5+((i*2)+0)] = GetDataMem(usAddress+(((i*2)+0)/2)).u8_data[((i*2)+0)%2 BE1];
		g_stModbusTCPMBuf.Mod_TX.Data[5+((i*2)+1)] = GetDataMem(usAddress+(((i*2)+1)/2)).u8_data[((i*2)+1)%2 BE1];
	}

	g_stModbusTCPMBuf.Mod_TX.Len = 7+(usQuantity*2); // Set Length (Unit ID + Func Code + Start addr(2) + Quantity(2) + Byte Count + Data[Byte_Count])
	g_stModbusTCPMBuf.Rx_New_Data = 0; // Clear flag Rx new data.

	return 1;
}


/* --------------------------------------------------------
���� :
���ϰ� :
Function code FC-16: Write Multiple Registers ����
-------------------------------------------------------- */
s32 Process_FC16_M_Res()
{
	s32 i;

	for(i=0; i<4; i++)
	{
		if(g_stModbusTCPMBuf.Mod_TX.Data[i] != g_stModbusTCPMBuf.Mod_RX.Data[i])
		{
			printf("g_stModbusTCPMBuf.Mod_TX.Data[i] != g_stModbusTCPMBuf.Mod_RX.Data[i]\r\n");
			return -1;
		}
	}

	return 1;
}


/* --------------------------------------------------------
���� :
���ϰ� :
Modbus tcp master request ó��
-------------------------------------------------------- */
s32 Process_Modbus_TCP_M_Req(u8 ucMaster)
{
	ModbusTcpMCfgType stModbusTcpMCfg;
	s32 i, iRet=0;
	u8 ucQuery = GetQuery(ucMaster);

	ModbusTCP_M_CfgGet(ucMaster, &stModbusTcpMCfg);

	g_stModbusTCPMBuf.Mod_TX.TID = 0; // Transaction ID
	g_stModbusTCPMBuf.Mod_TX.PID = 0; // Protocol ID
	g_stModbusTCPMBuf.Mod_TX.UID = stModbusTcpMCfg.ucSlaveID; // Unit ID

	g_stModbusTCPMBuf.Mod_TX.Func = (u8)stModbusTcpMCfg.Query[ucQuery].ucFunction;
	switch( g_stModbusTCPMBuf.Mod_TX.Func )
	{
		case FC3: // Read Multiple Registers
			iRet = Process_FC3_M_Req(ucMaster);
			break;

		case FC16: // Write Multiple Registers
			iRet = Process_FC16_M_Req(ucMaster);
			break;

		default: // Undefined Function.
			break;
	}

	if(iRet > 0) // Make Send Modbus Packet
	{
		g_stModbusTCPMBuf.Send_Modbus_Packet[0] = (u8)(g_stModbusTCPMBuf.Mod_TX.TID >> 8); // Transaction ID
		g_stModbusTCPMBuf.Send_Modbus_Packet[1] = (u8)(g_stModbusTCPMBuf.Mod_TX.TID & 0x00FF);
		g_stModbusTCPMBuf.Send_Modbus_Packet[2] = (u8)(g_stModbusTCPMBuf.Mod_TX.PID >> 8); // Protocol ID
		g_stModbusTCPMBuf.Send_Modbus_Packet[3] = (u8)(g_stModbusTCPMBuf.Mod_TX.PID & 0x00FF);
		g_stModbusTCPMBuf.Send_Modbus_Packet[4] = (u8)(g_stModbusTCPMBuf.Mod_TX.Len >> 8); // Length
		g_stModbusTCPMBuf.Send_Modbus_Packet[5] = (u8)(g_stModbusTCPMBuf.Mod_TX.Len & 0x00FF);
		g_stModbusTCPMBuf.Send_Modbus_Packet[6] = g_stModbusTCPMBuf.Mod_TX.UID; // Unit ID
		g_stModbusTCPMBuf.Send_Modbus_Packet[7] = g_stModbusTCPMBuf.Mod_TX.Func; // Function Code
		for(i=0; i<(g_stModbusTCPMBuf.Mod_TX.Len-2); i++) // Data
		{
			g_stModbusTCPMBuf.Send_Modbus_Packet[8+i] = g_stModbusTCPMBuf.Mod_TX.Data[i];
		}
	}

	return iRet;
}


/* --------------------------------------------------------
���� :
���ϰ� :
Modbus tcp master response ó��
-------------------------------------------------------- */
s32 Process_Modbus_TCP_M_Res()
{
	s32 iRet=0;

	switch( g_stModbusTCPMBuf.Mod_RX.Func ) // Process Function Code
	{
		case FC3: // Read Multiple Registers
			iRet = Process_FC3_M_Res();
			break;

		case FC16: // Write Multiple Registers
			iRet = Process_FC16_M_Res();
			break;

		default: // Undefined Function.
			printf("Undefined Function. = %d\r\n", g_stModbusTCPMBuf.Mod_RX.Func);
			break;
	}

	return iRet;
}


// ====================================================================== //
//	Function: Modbus_Rx_Buffer
//		- Save the Modbus-TCP Packet.
// ---------------------------------------------------------------------- //
void Modbus_Rx_M_Buffer(u8* TCP_Rx_data, u16 TCP_Rx_data_len)
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
		g_stModbusTCPMBuf.Mod_RX.TID = temp16;

		// Protocol ID
		temp8_1 = *(TCP_Rx_data+2);
		temp8_2 = *(TCP_Rx_data+3);
		temp16 = (((u16)(temp8_1))<<8)+(u16)(temp8_2);
		g_stModbusTCPMBuf.Mod_RX.PID = temp16;

		// Length
		temp8_1 = *(TCP_Rx_data+4);
		temp8_2 = *(TCP_Rx_data+5);
		temp16 = (((u16)(temp8_1))<<8)+(u16)(temp8_2);
		g_stModbusTCPMBuf.Mod_RX.Len = temp16;

		// Uint ID
		g_stModbusTCPMBuf.Mod_RX.UID = *(TCP_Rx_data+6);

		// Function Code
		g_stModbusTCPMBuf.Mod_RX.Func = *(TCP_Rx_data+7);//ucaInFrame[1]

		// Data
		for ( i=0; i < (g_stModbusTCPMBuf.Mod_RX.Len-2); i++)
		{
			g_stModbusTCPMBuf.Mod_RX.Data[i] = *(TCP_Rx_data+(8+i));//ucaInFrame[2~]
		}

		g_stModbusTCPMBuf.Rx_New_Data = 1; // Set flag Rx new data.
	}
	else
	{
		g_stModbusTCPMBuf.Rx_New_Data = 0; // Modbus-TCP Packet is net received.
	}
}
