#include "BPuck2Manager.h"

#include "BPuck2.h"
#include "bhand_misc.h"

#include "puck2.h"


// Some defines that affect the wakeup of pucks
#define PUCK2_WAKEUP_MAX_TRIES 3
#define PUCK2_WAKEUP_DELAY 1000


///////////////////////////////////////////////////////////////////////////////
// BPuck2Manager instance methods
///////////////////////////////////////////////////////////////////////////////

BPuck2Manager::BPuck2Manager(unsigned int fromID) :
	m_fromID(fromID), bypassWakeup(false)
{
	for (int i = 0; i <= PUCK2_ID_MAX; i++)
	{
		pucks[i] = NULL;
		wakenUp[i] = false;
	}
}

BPuck2Manager::~BPuck2Manager()
{
	for (int i = 0; i <= PUCK2_ID_MAX; i++)
	{
		if (pucks[i] != NULL)
			delete pucks[i];
	}
}

int BPuck2Manager::init()
{
	return puck2CANInit();
}

void BPuck2Manager::close()
{
	puck2CANClose();
}


int BPuck2Manager::addPuck(unsigned int puckID)
{
	// Check that there are no problems to add Puck2 instance to Puck2 collection
	if (puckID > PUCK2_ID_MAX || pucks[puckID] != NULL)
		return -1;

	pucks[puckID] = new BPuck2(this, puckID);

	return 0;
}

BPuck2 * BPuck2Manager::getPuck(unsigned int puckID)
{
	return puckID <= PUCK2_ID_MAX ? pucks[puckID] : NULL;
}


int BPuck2Manager::setPropertyUInt32(unsigned int puckID, unsigned char puckProperty, unsigned int value, bool verify)
{
	//printf("set puck %d property %d val %d\n", puckID, puckProperty, value);
	return (puckID <= PUCK2_ID_MAX && (wakenUp[puckID] || bypassWakeup)) ? setPuckPropertyUInt32(puckID, puckProperty, value, verify, m_fromID) : -31;
}
int BPuck2Manager::setPropertyUInt16(unsigned int puckID, unsigned char puckProperty, unsigned short value, bool verify)
{
	//printf("set puck %d property %d val %d\n", puckID, puckProperty, value);
	return (puckID <= PUCK2_ID_MAX && (wakenUp[puckID] || bypassWakeup)) ? setPuckPropertyUInt16(puckID, puckProperty, value, verify, m_fromID) : -31;
}
int BPuck2Manager::setPropertyUInt8(unsigned int puckID, unsigned char puckProperty, unsigned char value, bool verify)
{
	//printf("set puck %d property %d val %d\n", puckID, puckProperty, value);
	return (puckID <= PUCK2_ID_MAX && (wakenUp[puckID] || bypassWakeup)) ? setPuckPropertyUInt8(puckID, puckProperty, value, verify, m_fromID) : -31;
}


int BPuck2Manager::getPropertyUInt32(unsigned int puckID, unsigned char puckProperty, unsigned int *result)
{
	return (puckID <= PUCK2_ID_MAX && (wakenUp[puckID] || bypassWakeup)) ? getPuckPropertyUInt32(puckID, puckProperty, result) : -31;
}
int BPuck2Manager::getPropertyUInt16(unsigned int puckID, unsigned char puckProperty, unsigned short *result)
{
	return (puckID <= PUCK2_ID_MAX && (wakenUp[puckID] || bypassWakeup)) ? getPuckPropertyUInt16(puckID, puckProperty, result) : -31;
}
int BPuck2Manager::getPropertyUInt8(unsigned int puckID, unsigned char puckProperty, unsigned char *result)
{
	return (puckID <= PUCK2_ID_MAX && (wakenUp[puckID] || bypassWakeup)) ? getPuckPropertyUInt8(puckID, puckProperty, result) : -31;
}


int BPuck2Manager::readMsgAsync(int *puckID, unsigned char *data, int *dataLength)
{
	int result = readCANMsg(puckID, dataLength, data, true);
	//return (*puckID >= 0 && *puckID <= PUCK2_ID_MAX && (wakenUp[*puckID] || bypassWakeup) && result == 0) ? 0 : -30;
	return result;
}


void BPuck2Manager::startGetPropertyBatch()
{
	batchGetSize = 0;
}

int BPuck2Manager::addToGetPropertyBatch(unsigned int puckID, unsigned int *result)
{
	if (puckID <= PUCK2_ID_MAX && (wakenUp[puckID] || bypassWakeup) && batchGetSize <= PUCK2_ID_MAX)
	{
		batchGetPuckIDs[batchGetSize] = puckID;
		batchGetPuckResults[batchGetSize] = result;
		// *result = 0; // let the caller initialize the destination instead (in case of errors, result will be unchanged)
		batchGetSize++;
		return 0;
	}
	else
		return -32;
}


int BPuck2Manager::getBatchPropertyUInt32(unsigned int puckProperty)
{
	if (batchGetSize == 0)
		return 0;

	int err;
	if ((err = puckBatchGet(batchGetPuckIDs, batchGetSize, puckProperty)))
		return err;

	//printf("puckBatchGet...\n");
	if (!(err = puckBatchGetResultsUInt32()))
	{
		// All results received - no partial success is handled
		for (unsigned int i = 0; i < batchGetSize; i++)
			*(batchGetPuckResults[i]) = getPuckBatchResult(batchGetPuckIDs[i]);
		//printf("puckBatchGetResultsUInt32 success\n");

		return 0;
	}
	else
	{
		//printf("puckBatchGetResultsUInt32 err\n");
		return err;
	}
}


int BPuck2Manager::awaken()
{
	for (int i = 0; i <= PUCK2_ID_MAX; i++)
		wakenUp[i] = false;
	bypassWakeup = true;
	for (int tries = 1; tries <= PUCK2_WAKEUP_MAX_TRIES; tries++)
	{
		bool allAwaken = true;

		// Go through and wake up all pucks and determine if any Puck is not up
		for (int i = 0; i <= PUCK2_ID_MAX; i++)
		{
			// Skip waking up this puck?
			if (pucks[i] == NULL)
				continue;

			// Get Puck Status Property to determine if the Puck is up or not
			unsigned int statusProperty = 0;
			int r = pucks[i]->getPropertyUInt32(PROP_STAT, &statusProperty);

			//printf("get puck %d property %d val %d\n", i, PROP_STAT, statusProperty);

			// Check Puck Status Property and mark puck as up if it is
			if (!r && statusProperty == 2)
				wakenUp[i] = true;
			else
			{
				allAwaken = false;
				// Set Puck Status Property to wake up each Puck one at a time
				pucks[i]->setPropertyUInt8(PROP_STAT, 2);
				// not helpful anymore
				//break; // shortens the maximum delay - but prevents higher number pucks from waking up
			}
		}

		// If any Puck still needs to wake up delay a bit
		if (allAwaken)
		{
			bypassWakeup = false;
			return 0;
		}
		else if (tries < PUCK2_WAKEUP_MAX_TRIES)
		{
			DELAY(PUCK2_WAKEUP_DELAY);
		}
	}

	bypassWakeup = false;

	return -1;
}


void BPuck2Manager::putToSleep(unsigned int puckID)
{
	if (puckID < PUCK2_ID_MAX)
		wakenUp[puckID] = false;
}

bool BPuck2Manager::awake(unsigned int puckID)
{
	return wakenUp[puckID];
}
