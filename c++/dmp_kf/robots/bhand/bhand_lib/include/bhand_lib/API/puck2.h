#ifndef PUCK2_H
#define PUCK2_H


#define PUCK2_ID_MAX 31

#define PUCK2_ID_PC 6


/* Command enumeration. Append only. */
enum {
	CMD_LOAD, CMD_SAVE,	CMD_RESET, CMD_DEF, CMD_GET, CMD_FIND, CMD_SET, CMD_HOME,
	CMD_KEEP, CMD_LOOP, CMD_PASS, CMD_VERS, CMD_ERR, CMD_HI, CMD_IC, CMD_IO,
	CMD_TC, CMD_TO, CMD_C, CMD_M, CMD_O, CMD_T, CMD_HELP, CMD_END
};


















































































/* Common */
enum {
	PROP_VERS = 0,
	PROP_ROLE, // P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR
	PROP_SN,
	PROP_ID,
	PROP_ERROR,
	PROP_STAT,
	PROP_ADDR,
	PROP_VALUE,
	PROP_MODE,
	PROP_TEMP,
	PROP_PTEMP,
	PROP_OTEMP,
	PROP_BAUD,
	PROP_LOCK,
	PROP_DIG0,
	PROP_DIG1,
	PROP_FET0,
	PROP_FET1,
	PROP_ANA0,
	PROP_ANA1,
	PROP_THERM,
	PROP_VBUS,
	PROP_IMOTOR,
	PROP_VLOGIC,
	PROP_ILOGIC,
	PROP_SG,
	PROP_GRPA,
	PROP_GRPB,
	PROP_GRPC,
	PROP_CMD, // For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T
	PROP_SAVE,
	PROP_LOAD,
	PROP_DEF,
	PROP_FIND,
	PROP_X0,
	PROP_X1,
	PROP_X2,
	PROP_X3,
	PROP_X4,
	PROP_X5,
	PROP_X6,
	PROP_X7,

	COMMON_END // 42
};



/*
// Safety
enum {
	PROP_ZERO = COMMON_END,
	PROP_PEN,
	PROP_SAFE,
	PROP_VL1,
	PROP_VL2,
	PROP_TL1,
	PROP_TL2,
	PROP_VOLTL1,
	PROP_VOLTL2,
	PROP_VOLTH1,
	PROP_VOLTH2,
	PROP_PWR,
	PROP_MAXPWR,
	PROP_IFAULT,
	PROP_VNOM,

	SAFETY_END // 57
};*/







































































/* Tater */
enum {
	PROP_T = COMMON_END,
	PROP_MT,
	PROP_V,
	PROP_MV,
	PROP_MCV,
	PROP_MOV,
	PROP_P, /* 32-Bit Present Position */
	PROP_P2,
	PROP_DP, /* 32-Bit Default Position */
	PROP_DP2,
	PROP_E, /* 32-Bit Endpoint */
	PROP_E2,
	PROP_OT, /* 32-Bit Open Target */
	PROP_OT2,
	PROP_CT, /* 32-Bit Close Target */
	PROP_CT2,
	PROP_M, /* 32-Bit Move command for CAN*/
	PROP_M2,
	PROP_DS,
	PROP_MOFST,
	PROP_IOFST,
	PROP_UPSECS,
	PROP_OD,
	PROP_MDS,
	PROP_MECH, /* 32-Bit */
	PROP_MECH2,
	PROP_CTS, /* 32-Bit */
	PROP_CTS2,
	PROP_PIDX,
	PROP_HSG,
	PROP_LSG,
	PROP_IVEL,
	PROP_IOFF, /* 32-Bit */
	PROP_IOFF2,
	PROP_MPE,
	PROP_HOLD,
	PROP_TSTOP,
	PROP_KP,
	PROP_KD,
	PROP_KI,
	PROP_ACCEL,
	PROP_TENST,
	PROP_TENSO,
	PROP_JIDX,
	PROP_IPNM,
	PROP_HALLS,
	PROP_HALLH, /* 32-Bit */
	PROP_HALLH2,
	PROP_POLES,
	PROP_IKP,
	PROP_IKI,
	PROP_IKCOR,
	PROP_EN, /* 32-Bit */
	PROP_EN2,
	PROP_JP, /* 32-Bit */
	PROP_JP2,
	PROP_JOFST, /* 32-Bit */
	PROP_JOFST2,
	PROP_TIE,
	PROP_ECMAX,
	PROP_ECMIN,
	PROP_LFLAGS,
	PROP_LCTC,
	PROP_LCVC,
	PROP_TACT,
	PROP_TACTID,

	PROP_END
};


///////////////////////////////////////////////////////////////////////////
// Property Accessor Methods
///////////////////////////////////////////////////////////////////////////

const char * puck2PropertyName(unsigned int index);

///////////////////////////////////////////////////////////////////////////
// Init/Close methods for CAN interface
///////////////////////////////////////////////////////////////////////////

int puck2CANInit();
void puck2CANClose();


///////////////////////////////////////////////////////////////////////////
// Methods for sending/recieving CAN messages
///////////////////////////////////////////////////////////////////////////

int sendCANMsg(bool group, unsigned int from, unsigned int to, unsigned int length, const unsigned char *data);
int readCANMsg(int *id, int *len, unsigned char *data, bool skipCheckID = false);


///////////////////////////////////////////////////////////////////////////
// Low-Level methods to Get/Set Puck Properties
///////////////////////////////////////////////////////////////////////////

int setPuckPropertyUInt32(unsigned int puckID, unsigned char puckProperty, unsigned int value, bool verify = 0, int who = PUCK2_ID_PC);
int setPuckPropertyUInt16(unsigned int puckID, unsigned char puckProperty, unsigned short value, bool verify = 0, int who = PUCK2_ID_PC);
int setPuckPropertyUInt8(unsigned int puckID, unsigned char puckProperty, unsigned char value, bool verify = 0, int who = PUCK2_ID_PC);

int getPuckPropertyUInt8(unsigned int puckID, unsigned char puckProperty, unsigned char *result);
int getPuckPropertyUInt16(unsigned int puckID, unsigned char puckProperty, unsigned short *result);
int getPuckPropertyUInt32(unsigned int puckID, unsigned char puckProperty, unsigned int *result);

int puckBatchGet(unsigned int *puckIDs, unsigned char numPucks, unsigned char puckProperty);
int puckBatchGetResultsUInt32();
unsigned int getPuckBatchResult(unsigned int puckID);

#endif
