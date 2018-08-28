#include "PucksInHand.h"

// Pointer that will need to be set in order to execute helper methods contained in this file
// This variable must be declared as an extern and should be initialized prior to calling
// Pucks In Hands Supervisory or RealTime Methods
//extern BHAND_PUCK2 *bhandPuckTwos;


#include <string.h>


// Number of default properties
#define BHAND_NUM_DEFAULTS 31
#define BHAND_NUM_SPREAD_DEFAULTS 6


struct defaultStruct
{
   int prop;
   long val;
};


// Pointer that will need to be set in order to execute supervisory commands contained in this file
// This variable must be declared as an extern and should be initialized prior to calling
// Pucks In Hands Supervisory or RealTime Methods
//extern BHAND_PUCK2 *bhandPuckTwos;

static struct defaultStruct bh8Defs[BHAND_NUM_DEFAULTS] =
{
   {PROP_TIE, 0},
   {PROP_ACCEL, 200},
   {PROP_P, 0},
   {PROP_CT, 195000},
   {PROP_OT, 0},

   {PROP_CTS, 4096},
   {PROP_DP, 45000},
   {PROP_MT, 3300},
   {PROP_MV, 200},
   {PROP_MCV, 200},

   {PROP_MOV, 200},
   {PROP_HOLD, 0},
   {PROP_TSTOP, BHAND_DEFAULT_TSTOP},
   {PROP_OTEMP, 60},

   {PROP_PTEMP, 0},
   {PROP_POLES, 6},
   {PROP_IKI, 204},
   {PROP_IKP, 500},
   {PROP_IKCOR, 102},

   {PROP_IOFF, 0},
   {PROP_IVEL, -75},
   {PROP_DS, 25600},
   {PROP_KP, 500},
   {PROP_KD, 5000},

   {PROP_KI, 0},
   {PROP_IPNM, 20000},
   {PROP_HSG, 0},
   {PROP_LSG, 0},
   {PROP_GRPA, 0},

   {PROP_GRPB, 7},
   {PROP_GRPC, 8}
};

static struct defaultStruct bh8DefsForSpread[BHAND_NUM_SPREAD_DEFAULTS] =
{
	{PROP_CT, 35950},
	{PROP_DP, 17975},
	{PROP_MV, 50},
	{PROP_HSG, 0},
	{PROP_LSG, 0},
	{PROP_HOLD, 1}
};

///////////////////////////////////////////////////////////////////////////////
// Accessor methods
///////////////////////////////////////////////////////////////////////////////

int pucksInHandPropertyNumDefault() { return BHAND_NUM_DEFAULTS; }
int pucksInHandPropertyNumSpreadDefault() { return BHAND_NUM_SPREAD_DEFAULTS; }

void pucksInHandPropertyDefault(int index, int *property, int *value)
{
	*property = bh8Defs[index].prop;
	*value = bh8Defs[index].val;
}
void pucksInHandPropertySpreadDefault(int index, int *property, int *value)
{
	*property = bh8DefsForSpread[index].prop;
	*value = bh8DefsForSpread[index].val;
}

