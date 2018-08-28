#include "puck2.h"

#include "bhand_parse.h"
#include "bhand_misc.h"

// PCAN includes for PCAN USB to CAN adapter
#ifdef LINUX
	// Linux
	#include <unistd.h>   // exit
	#include <string.h>
	#include <stdlib.h>   // strtoul
	#include <fcntl.h>    // O_RDWR
	#include <libpcan.h>
	#include <ctype.h>
#else
	// Windows
	#include "Pcan_usb.h"
#endif















































































const char puck2PropertyNames[PROP_END][32] = {
	// Common
	"VERS",
	"ROLE", // P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR
	"SN",
	"ID",
	"ERROR",
	"STAT",
	"ADDR",
	"VALUE",
	"MODE",
	"TEMP",
	"PTEMP",
	"OTEMP",
	"BAUD",
	"LOCK",
	"DIG0",
	"DIG1",
	"FET0",
	"FET1",
	"ANA0",
	"ANA1",
	"THERM",
	"VBUS",
	"IMOTOR",
	"VLOGIC",
	"ILOGIC",
	"SG",
	"GRPA",
	"GRPB",
	"GRPC",
	"CMD", // For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T
	"SAVE",
	"LOAD",
	"DEF",
	"FIND",
	"X0",
	"X1",
	"X2",
	"X3",
	"X4",
	"X5",
	"X6",
	"X7", // Tater
	"T", // = COMMON_END,
	"MT",
	"V",
	"MV",
	"MCV",
	"MOV",
	"P", /* 32-Bit Present Position */
	"P2",
	"DP", /* 32-Bit Default Position */
	"DP2",
	"E", /* 32-Bit Endpoint */
	"E2",
	"OT", /* 32-Bit Open Target */
	"OT2",
	"CT", /* 32-Bit Close Target */
	"CT2",
	"M", /* 32-Bit Move command for CAN*/
	"M2",
	"DS",
	"MOFST",
	"IOFST",
	"UPSECS",
	"OD",
	"MDS",
	"MECH", /* 32-Bit */
	"MECH2",
	"CTS", /* 32-Bit */
	"CTS2",
	"PIDX",
	"HSG",
	"LSG",
	"IVEL",
	"IOFF", /* 32-Bit */
	"IOFF2",
	"MPE",
	"HOLD",
	"TSTOP",
	"KP",
	"KD",
	"KI",
	"ACCEL",
	"TENST",
	"TENSO",
	"JIDX",
	"IPNM",
	"HALLS",
	"HALLH", /* 32-Bit */
	"HALLH2",
	"POLES",
	"IKP",
	"IKI",
	"IKCOR",
	"EN", /* 32-Bit */
	"EN2",
	"JP", /* 32-Bit */
	"JP2",
	"JOFST", /* 32-Bit */
	"JOFST2",
	"TIE",
	"ECMAX",
	"ECMIN",
	"LFLAGS",
	"LCTC",
	"LCVC",
	"TACT",
	"TACTID"
	};


#ifdef LINUX
	const char szDevNode[] = "/dev/pcanusb0";

	HANDLE h;
#else
	HANDLE recvEvent = NULL;
#endif


const char * puck2PropertyName(unsigned int index)
{
	return (index < PROP_END) ? puck2PropertyNames[index] : NULL;
}


// Some helper definitions for parsing the standard identifiers in CAN messages
#define GET_GROUP_BIT_FROM_SID(sid) (((sid) >> 10) & 1)
#define GET_FROM_FIELD_FROM_SID(sid) (((sid) >> 5) & 0x1f)
#define GET_TO_FIELD_FROM_SID(sid) ((sid) & 0x1f)


int puck2CANInit()
{
#ifdef LINUX

	h = LINUX_CAN_Open(szDevNode, O_RDWR);
	if (!h)
		return -90;

	// clear status
	CAN_Status(h);

	if (CAN_Init(h, CAN_BAUD_1M, CAN_INIT_TYPE_ST) != CAN_ERR_OK)
		return -100;

	// Set filter to receive all standard messages (from 0 to 0x3FF)
	//if (CAN_MsgFilter(h, 0, 0x3FF, MSGTYPE_STANDARD) != CAN_ERR_OK)
	//	return -102;

#else

	// Open CAN interface with the Barrett Hand
	// We want 1 Mbps communication and will only use the 11-bit SID message type
	if (CAN_Init(CAN_BAUD_1M, 0) != CAN_ERR_OK)
		return -100;

	// Clear the TX/RX queues
	if (CAN_ResetClient() != CAN_ERR_OK)
		return -101;

	// Set filter to receive all standard messages (from 0 to 0x3FF)
	if (CAN_MsgFilter(0, 0x3FF, MSGTYPE_STANDARD) != CAN_ERR_OK)
		return -102;

	// Create receive signal event object
	recvEvent = CreateEvent(
        NULL,                 // default security attributes
        FALSE,                // manual-reset event
        FALSE,                // initial state is nonsignaled
        TEXT("ReceiveEvent")  // object name
        );

	// Set CAN receive signal object handle
	if (CAN_SetRcvEvent(recvEvent) != CAN_ERR_OK)
		return -103;

#endif

	return 0;
}


void puck2CANClose()
{
#ifdef LINUX
	if (h) CAN_Close(h);
#else
	if (recvEvent != NULL)
	{
		CloseHandle(recvEvent);
		recvEvent = NULL;
	}
	CAN_Close();
#endif
}


int sendCANMsg(bool group, unsigned int from, unsigned int to, unsigned int length, const unsigned char *data)
{
	if (from > PUCK2_ID_MAX || to > PUCK2_ID_MAX)
		return -29; // just a non-zero number

	//printf("   sendCANMsg(%d, %d, %d, %d,  %d %d %d %d %d %d %d %d\n", group, from, to, length, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);


	// Generate the outbound message
	TPCANMsg msgOut;

	msgOut.ID = 0;
	msgOut.ID |= group ? (1 << 10) : 0;
	msgOut.ID |= (from & 0x1f) << 5;
	msgOut.ID |= to & 0x1f;

	msgOut.MSGTYPE = MSGTYPE_STANDARD;
	msgOut.LEN = length;

	memcpy(msgOut.DATA, data, length);

	// Send the message
#ifdef LINUX
	int res = CAN_Write(h, &msgOut);
#else
	int res = CAN_Write(&msgOut);
	while (res & CAN_ERR_BUSOFF)
	{
		// Check for CAN bus off error, must reinitialize to recover from this error
		printf("Alert: check that the hand is securely connected (with the termination switch\n");
		printf("set correctly) and that the power is on. Write errors codes: %d", res);
		res &= ~CAN_ERR_BUSOFF;
		puck2CANClose();
		DELAY(5); // wait for bus errors to settle
		res |= puck2CANInit();
		printf(" %d", res);
		res |= CAN_Write(&msgOut);
		printf(" %d\n", res);
	}
#endif

	//printf("   CAN_Write res = %d\n", res);

	return res;
}


int setPuckPropertyUInt32(unsigned int puckID, unsigned char puckProperty, unsigned int value, bool verify, int who)
{
	//printf("puck2.cpp setPuckPropertyUInt32(%d, %d, %d)\n", puckID, puckProperty, value);
	if (puckID > PUCK2_ID_MAX)
		return -30;

	unsigned char data[6] = { 0x80, 0, 0, 0, 0, 0 };

	data[0] |= puckProperty;
	data[2] = (unsigned char)((value & 0x000000ff));
	data[3] = (unsigned char)((value & 0x0000ff00) >> 8);
	data[4] = (unsigned char)((value & 0x00ff0000) >> 16);
	data[5] = (unsigned char)((value & 0xff000000) >> 24);

	// Send message over CAN to set Puck2 property
	int r = sendCANMsg(0, who, puckID, 6, data);

	if (!verify)
		return r;

	// Get puck property over CAN from Puck2
	unsigned int puckValue;
	r = getPuckPropertyUInt32(puckID, puckProperty, &puckValue);
	if (r)
		return r; // Get error - read result is non-zero

	// Get was successful - now compare values written
	return (puckValue == value) ? 0 : -1; // return 0 on successful verify
}


int setPuckPropertyUInt16(unsigned int puckID, unsigned char puckProperty, unsigned short value, bool verify, int who)
{
	if (puckID > PUCK2_ID_MAX)
		return -1;

	unsigned char data[] = { 0x80, 0, 0, 0 };

	data[0] |= puckProperty;
	data[2] |= value & 0xff;
	data[3] |= (value & 0xff00) >> 8;

	// Send message over CAN to set Puck2 property
	int r = sendCANMsg(0, who, puckID, 4, data);
	if (!verify)
		return r;

	// Get puck property over CAN from Puck2
	unsigned short puckValue;
	r = getPuckPropertyUInt16(puckID, puckProperty, &puckValue);
	if (r)
		return r; // Get error - read result is non-zero

	// Get was successful - now compare values written
	return (puckValue == value) ? 0 : -1; // return 0 on successful verify
}


int setPuckPropertyUInt8(unsigned int puckID, unsigned char puckProperty, unsigned char value, bool verify, int who)
{
	if (puckID > PUCK2_ID_MAX)
		return -1;

	unsigned char data[] = { 0x80, 0, 0, 0 };

	data[0] |= puckProperty;
	data[2] |= value;

	// Send message over CAN to set Puck2 property
	int r = sendCANMsg(0, who, puckID, 4, data);
	if (!verify)
		return r;

	// Get puck property over CAN from Puck2
	unsigned int puckValue;
	r = getPuckPropertyUInt32(puckID, puckProperty, &puckValue);
	if (r)
		return r; // Get error - read result is non-zero

	// Get was successful - now compare values written
	return (puckValue == value) ? 0 : -1; // return 0 on successful verify
}


int readCANMsg(int *id, int *len, unsigned char *data, bool skipCheckID)
{
	int err = 0;

	#ifdef LINUX
		TPCANRdMsg msgIn;
		//****************************************************************************
		//  LINUX_CAN_Read_Timeout()
		//  reads a message WITH TIMESTAMP from the CAN bus. If there is no message
		//  to read the current request blocks until either a new message arrives
		//  or a timeout or a error occures.
		//  nMicroSeconds  > 0 -> Timeout in microseconds
		//  nMicroSeconds == 0 -> polling
		//  nMicroSeconds  < 0 -> blocking, same as LINUX_CAN_Read()
		//  DWORD LINUX_CAN_Read_Timeout(HANDLE hHandle, TPCANRdMsg* pMsgBuff, int nMicroSeconds)
		err = LINUX_CAN_Read_Timeout(h, &msgIn, 1000);
	#else
		TPCANMsg msgIn;

		// Start a timer to keep track of a property read timeout
		BHiResTimer tmr;
		tmr.Start();

		// Check for property read timeout
		while (tmr.GetElapsedInMilliSecs() < 1000)
		{
			// Read a CAN message
			err = CAN_Read(&msgIn);

			if (err == CAN_ERR_OK)
				break;
			while (err & CAN_ERR_BUSOFF)
			{
				// Check for CAN bus off error, must reinitialize to recover from this error
				//printf("CAN_Read returning CAN_ERR_BUSOFF error %d\n", err);
				printf("Alert: check that the hand is securely connected (with the termination switch\n");
				printf("set correctly) and that the power is on. Read errors codes: %d", err);

				err &= ~CAN_ERR_BUSOFF;
				puck2CANClose();
				DELAY(5); // wait for bus errors to settle
				err |= puck2CANInit();
				//printf("puck2CANInit returning %d result\n", err);
				printf(" %d", err);
				err |= CAN_Read(&msgIn);
				//printf("CAN_Read now returning %d result\n", err);
				printf(" %d\n", err);
			}

			// Wait for the receive object to be signalled before checking for more received CAN messages
			WaitForSingleObject(recvEvent, 1);
			//ResetEvent(recvEvent); // event is set to auto reset
		}

	#endif


	if (err == CAN_ERR_OK)
	{
		// A new CAN message has been received
		#ifdef LINUX
		if (msgIn.Msg.MSGTYPE == MSGTYPE_STANDARD)
		#else
		if (msgIn.MSGTYPE == MSGTYPE_STANDARD)
		#endif
		{
			// Received a standard message - process some fields in the CAN message

			// Extract "from" field SID
			int toID;
			#ifdef LINUX
				toID = GET_TO_FIELD_FROM_SID(msgIn.Msg.ID);
				*id = GET_FROM_FIELD_FROM_SID(msgIn.Msg.ID);
				*len = msgIn.Msg.LEN;
				memcpy(data, msgIn.Msg.DATA, *len);
			#else
				toID = GET_TO_FIELD_FROM_SID(msgIn.ID);
				*id = GET_FROM_FIELD_FROM_SID(msgIn.ID);
				*len = msgIn.LEN;
				memcpy(data, msgIn.DATA, *len);
			#endif

			if (skipCheckID)
				return 0; // for reading PPC sensors, don't check
			// Check that message was targetted for the PC
			// Don't check this anymore
			if (toID == PUCK2_ID_PC)
				return 0;
		}
	}

	return -1; // CAN message not received
}


bool puckInBatch[PUCK2_ID_MAX + 1];
unsigned int puckNumInBatch;
unsigned int puckBatchResults[PUCK2_ID_MAX + 1]; // TODO: will need to store data and length fields instead
int puckBatchErr = -1;

int puckBatchGet(unsigned int *puckIDs, unsigned char numPucks, unsigned char puckProperty)
{
	unsigned int n;

	// Set default batch values
	puckBatchErr = 0;
	puckNumInBatch = 0;
	for (n = 0; n <= PUCK2_ID_MAX; n++)
	{
		puckInBatch[n] = false;
		puckBatchResults[n] = 0; // In case of any error, have all results set to 0 right away
	}

	// Send GetProperty messages
	for (n = 0; n < numPucks; n++)
	{
		unsigned int puckID = puckIDs[n];
		if (puckID <= PUCK2_ID_MAX && !puckInBatch[puckID])
		{
			puckInBatch[puckID] = true;

			if ((puckBatchErr = sendCANMsg(0, PUCK2_ID_PC, puckID, 1, &puckProperty)))
				return puckBatchErr;

			puckNumInBatch++;
			//printf("send msg to %d\n", puckID);
		}
		else
		{
			puckBatchErr = -10;
			return puckBatchErr; // PuckID too large or included more than once
		}
	}

	return 0;
}

int puckBatchGetResultsUInt32()
{
	if (puckBatchErr)
		return puckBatchErr;

	// Start a timer to keep track of a property read timeout
	BHiResTimer tmr;
	tmr.Start();

	while (1)
	{
		int id;
		int len;
		unsigned char data[8];
		if (!(puckBatchErr = readCANMsg(&id, &len, data)))
		{
			if (id >= 0 && id <= PUCK2_ID_MAX && puckInBatch[id])
			{
				// This message contains the property that we want (assumed)
				//printf("received msg from %d\n", id);

				int node;       // The controller node ID of the received message
				int property;   // The property this message applies to
				long value;     // The value of the property being processed

				// int parseMessage(
				//		// Input
				//		int id                      // The message ID
				//		int len                     // The data payload length
				//		unsigned char *messageData  // Pointer to the message data payload
				//		// Output
				//		int *node       // The controller node ID of the received message
				//		int *property   // The property this message applies to
				//		long *value     // The value of the property being processed

				// Note: id is "from" field instead of the message SID
				parseMessage(id, len, data, &node, &property, &value); // node is not the value expected

				// Save property value in result
				// printf("property = %d puckProperty = %d data[0] = %d\n", property, puckProperty, data[0]); // these differ by 1 - don't rely on property either
				puckBatchResults[id] = (unsigned int)value;
				puckInBatch[id] = false; // Successfully received and parsed message
				puckNumInBatch--;

				if (puckNumInBatch == 0)
					return 0;
			}
			else
			{
				// Ignore unknown received message
			}
		}

		// Check for property read timeout
		if (tmr.GetElapsedInMilliSecs() > 1000)
		{
			puckBatchErr = -1;
			return -1;
		}
	}
}

unsigned int getPuckBatchResult(unsigned int puckID)
{
	return puckBatchResults[puckID];
}


int getPuckPropertyUInt32(unsigned int puckID, unsigned char puckProperty, unsigned int *result)
{
	// In case of any error, have result set to 0 right away
	*result = 0;

	//printf("getPuckPropertyUInt32(%d, %d, ...)\n", puckID, puckProperty);

	if (puckID > PUCK2_ID_MAX)
		return -1;

	int err = 0;

	// Send the puck a Get Property message
	if ((err = sendCANMsg(0, PUCK2_ID_PC, puckID, 1, &puckProperty)))
		return err;


	// Start a timer to keep track of a property read timeout
	BHiResTimer tmr;
	tmr.Start();

	while (1)
	{
		int id;
		int len;
		unsigned char data[8];
		if (!(err = readCANMsg(&id, &len, data)))
		{
			//printf("   readCANMsg(%d, %d, ...) data = %d %d %d %d %d %d %d %d\n", id, len, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
			// Check that the returned property is from right Puck
			if ((unsigned char)id == puckID)
			{
				// This message contains the property that we want (assumed)

				int node;       // The controller node ID of the received message
				int property;   // The property this message applies to
				long value;     // The value of the property being processed

				// int parseMessage(
				//		// Input
				//		int id                      // The message ID
				//		int len                     // The data payload length
				//		unsigned char *messageData  // Pointer to the message data payload
				//		// Output
				//		int *node       // The controller node ID of the received message
				//		int *property   // The property this message applies to
				//		long *value     // The value of the property being processed

				// Note: id is "from" field instead of the message SID
				parseMessage(id, len, data, &node, &property, &value); // node is not the value expected

				// Save property value in result
				// printf("property = %d puckProperty = %d data[0] = %d\n", property, puckProperty, data[0]); // these differ by 1 - don't rely on property either
				*result = (unsigned int)value;

				return 0;
			}


		}

		// Check for property read timeout
		if (tmr.GetElapsedInMilliSecs() > 1000)
		{
			//printf("getPuckPropertyUInt32() timeout\n");
			return -1;
		}
	}

	//printf("No msg returned\n");

	return err;
}

int getPuckPropertyUInt16(unsigned int puckID, unsigned char puckProperty, unsigned short *result)
{
	unsigned int result_32;
	int err = getPuckPropertyUInt32(puckID, puckProperty, &result_32);

	*result = (unsigned short)result_32;

	return err;
}

int getPuckPropertyUInt8(unsigned int puckID, unsigned char puckProperty, unsigned char *result)
{
	unsigned int result_32;
	int err = getPuckPropertyUInt32(puckID, puckProperty, &result_32);

	*result = (unsigned char)result_32;

	return err;
}

/*int setProperty(int bus, int who, int property, int verify, long value)
{
	// For now, ignore "bus" and "verify" parameters
	return setPuckPropertyUInt32(who, property, value);
}*/
