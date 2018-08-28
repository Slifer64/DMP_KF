#include "BPuck2.h"

#include "BPuck2Manager.h"

BPuck2::BPuck2(BPuck2Manager *puckManager, unsigned int puckID) :
	m_puckManager(puckManager), m_puckID(puckID)
{
}

BPuck2::~BPuck2()
{
}

bool BPuck2::awake()
{
	return m_puckManager->awake(m_puckID);
}

int BPuck2::setPropertyUInt32(unsigned char puckProperty, unsigned int value, bool verify)
{
	return m_puckManager->setPropertyUInt32(m_puckID, puckProperty, value, verify);
}
int BPuck2::setPropertyUInt16(unsigned char puckProperty, unsigned short value, bool verify)
{
	return m_puckManager->setPropertyUInt16(m_puckID, puckProperty, value, verify);
}
int BPuck2::setPropertyUInt8(unsigned char puckProperty, unsigned char value, bool verify)
{
	return m_puckManager->setPropertyUInt8(m_puckID, puckProperty, value, verify);
}

int BPuck2::getPropertyUInt32(unsigned char puckProperty, unsigned int *result)
{
	return m_puckManager->getPropertyUInt32(m_puckID, puckProperty, result);
}
int BPuck2::getPropertyUInt16(unsigned char puckProperty, unsigned short *result)
{
	return m_puckManager->getPropertyUInt16(m_puckID, puckProperty, result);
}
int BPuck2::getPropertyUInt8(unsigned char puckProperty, unsigned char *result)
{
	return m_puckManager->getPropertyUInt8(m_puckID, puckProperty, result);
}

int BPuck2::addToGetPropertyBatch(unsigned int *result)
{
	return m_puckManager->addToGetPropertyBatch(m_puckID, result);
}
