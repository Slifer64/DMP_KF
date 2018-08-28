#ifndef BPUCK2_MANAGER_H
#define BPUCK2_MANAGER_H

#ifndef PUCK2_ID_MAX
#define PUCK2_ID_MAX 31
#endif

class BPuck2;

class BPuck2Manager
{
public:

	BPuck2Manager(unsigned int fromID);
	virtual ~BPuck2Manager();

	int init();

	void close();

	int addPuck(unsigned int puckID);

	BPuck2 * getPuck(unsigned int puckID);

	int awaken();
	void putToSleep(unsigned int puckID);
	bool awake(unsigned int puckID);

	int setPropertyUInt32(unsigned int puckID, unsigned char puckProperty, unsigned int value, bool verify = false);
	int setPropertyUInt16(unsigned int puckID, unsigned char puckProperty, unsigned short value, bool verify = false);
	int setPropertyUInt8(unsigned int puckID, unsigned char puckProperty, unsigned char value, bool verify = false);

	int getPropertyUInt32(unsigned int puckID, unsigned char puckProperty, unsigned int *result);
	int getPropertyUInt16(unsigned int puckID, unsigned char puckProperty, unsigned short *result);
	int getPropertyUInt8(unsigned int puckID, unsigned char puckProperty, unsigned char *result);

	void startGetPropertyBatch();
	int addToGetPropertyBatch(unsigned int puckID, unsigned int *result);
	int getBatchPropertyUInt32(unsigned int puckProperty);

	int readMsgAsync(int *puckID, unsigned char *data, int *dataLength);

private:

	unsigned int m_fromID;

	BPuck2 *pucks[PUCK2_ID_MAX + 1];
	bool wakenUp[PUCK2_ID_MAX + 1];
	bool bypassWakeup;

	unsigned int batchGetPuckIDs[PUCK2_ID_MAX + 1];
	unsigned int *batchGetPuckResults[PUCK2_ID_MAX + 1];
	unsigned int batchGetSize;

};

#endif
