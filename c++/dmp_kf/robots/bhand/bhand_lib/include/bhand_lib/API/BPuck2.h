#ifndef BPUCK2_H
#define BPUCK2_H


class BPuck2Manager;

class BPuck2
{
public:

	BPuck2(BPuck2Manager *puckManager, unsigned int puckID);
	virtual ~BPuck2();

	unsigned int getID() { return m_puckID; }

	bool awake();

	int setPropertyUInt32(unsigned char puckProperty, unsigned int value, bool verify = false);
	int setPropertyUInt16(unsigned char puckProperty, unsigned short value, bool verify = false);
	int setPropertyUInt8(unsigned char puckProperty, unsigned char value, bool verify = false);

	int getPropertyUInt32(unsigned char puckProperty, unsigned int *result);
	int getPropertyUInt16(unsigned char puckProperty, unsigned short *result);
	int getPropertyUInt8(unsigned char puckProperty, unsigned char *result);

	int addToGetPropertyBatch(unsigned int *result);

private:

	BPuck2Manager *m_puckManager;

	unsigned int m_puckID;

};


#endif
