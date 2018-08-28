#ifndef BHAND_HARDWARE_H
#define BHAND_HARDWARE_H

// TODO: Declare one or more of these to include BarrettHand Hardware Descriptions
#define BH8_262_HARDWARE
#define BH8_280_HARDWARE


#include <string.h>

// Forward Declaration
class BHandHardware;


// Constants enumerated for different BarrettHands (append only)
enum BHHardwareModel
{
	BH_262,
	BH_280
};


// Methods for getting access to BarrettHand hardware definitions
void bhandCreateHardwareDesc();
void bhandDestroyHardwareDesc();

unsigned int bhandGetHardwareNumModels();
BHandHardware * bhandGetHardwareDesc(unsigned int hwIndex);



// the number of extended attributes is equal to the size of the extended attribute list
typedef struct ExtendedAttributesStruct
{
	int minValue;
	int maxValue;
} ExtendedAttributes;


const ExtendedAttributes r1 = {0, 1};
const ExtendedAttributes r8 = {0, 255};
const ExtendedAttributes r12 = {0, 4095};
const ExtendedAttributes r15 = {0, 32767};
const ExtendedAttributes r16 = {0, 65535};
const ExtendedAttributes r16s = {-32768, 32767};
const ExtendedAttributes r32 = {0, 0xffffffff};

const ExtendedAttributes r0_2 = {0, 2};
const ExtendedAttributes r15_255 = {15, 255};
const ExtendedAttributes r0_256 = {0, 256};
const ExtendedAttributes r6_384 = {6, 384};
const ExtendedAttributes r_550_1250 = {-550, 1250};
const ExtendedAttributes r0_1250 = {0, 1250};
const ExtendedAttributes r0_3150 = {0, 3150};
const ExtendedAttributes r16_4080 = {16, 4080};
const ExtendedAttributes r0_17500 = {0, 17500};
const ExtendedAttributes r0_20000 = {0, 20000};

const ExtendedAttributes r280finger = {0, 200000};
const ExtendedAttributes r280spread = {0, 36000};

const ExtendedAttributes r262finger = {0, 18000};
const ExtendedAttributes r262spread = {0, 3150};


class BHandProperty
{
public:
	BHandProperty(const char *category, const char *name, const char *desc, bool readOnly,
		ExtendedAttributes extAttributes,
		const char *purpose, const char *notes)
	{
		m_category = new char[strlen(category) + 1]; strcpy(m_category, category);
		m_name = new char[strlen(name) + 1]; strcpy(m_name, name);
		m_desc = new char[strlen(desc) + 1]; strcpy(m_desc, desc);
		m_global = true; //(numExtAttributes == 1); //global;
		m_readOnly = readOnly;
		m_numExtendedAttributes = 1;
		m_extendedAttributes = new ExtendedAttributes[m_numExtendedAttributes];
		//for (int i = 0; i < numExtAttributes; i++)
		//{
			m_extendedAttributes[0].minValue = extAttributes.minValue;
			m_extendedAttributes[0].maxValue = extAttributes.maxValue;
		//}
		m_purpose = new char[strlen(purpose) + 1]; strcpy(m_purpose, purpose);
		m_notes = new char[strlen(notes) + 1]; strcpy(m_notes, notes);
	}
	BHandProperty(const char *category, const char *name, const char *desc, bool readOnly,
		ExtendedAttributes extAttributes1,
		ExtendedAttributes extAttributes2,
		ExtendedAttributes extAttributes3,
		ExtendedAttributes extAttributes4,
		const char *purpose, const char *notes)
	{
		m_category = new char[strlen(category) + 1]; strcpy(m_category, category);
		m_name = new char[strlen(name) + 1]; strcpy(m_name, name);
		m_desc = new char[strlen(desc) + 1]; strcpy(m_desc, desc);
		m_global = false; //(numExtAttributes == 1); //global;
		m_readOnly = readOnly;
		m_numExtendedAttributes = 4;
		m_extendedAttributes = new ExtendedAttributes[m_numExtendedAttributes];
		//for (int i = 0; i < numExtAttributes; i++)
		//{
			m_extendedAttributes[0].minValue = extAttributes1.minValue;
			m_extendedAttributes[0].maxValue = extAttributes1.maxValue;
			m_extendedAttributes[1].minValue = extAttributes2.minValue;
			m_extendedAttributes[1].maxValue = extAttributes2.maxValue;
			m_extendedAttributes[2].minValue = extAttributes3.minValue;
			m_extendedAttributes[2].maxValue = extAttributes3.maxValue;
			m_extendedAttributes[3].minValue = extAttributes4.minValue;
			m_extendedAttributes[3].maxValue = extAttributes4.maxValue;
		//}
		m_purpose = new char[strlen(purpose) + 1]; strcpy(m_purpose, purpose);
		m_notes = new char[strlen(notes) + 1]; strcpy(m_notes, notes);
	}

	virtual ~BHandProperty()
	{
		delete[] m_category;
		delete[] m_name;
		delete[] m_desc;
		delete[] m_extendedAttributes;
		delete[] m_purpose;
		delete[] m_notes;
	}

	const char * getCategory() { return m_category; }

	const char * getName() { return m_name; }
	const char * getDesc() { return m_desc; }

	bool isGlobal() { return m_global; }
	bool isReadOnly() { return m_readOnly; }

	unsigned int getNumExtendedAttributes() { return m_numExtendedAttributes; }

	int getMinValue(int index = 0) { return m_extendedAttributes[index].minValue; }
	int getMaxValue(int index = 0) { return m_extendedAttributes[index].maxValue; }

	const char * getPurpose() { return m_purpose; }
	const char * getNotes() { return m_notes; }

private:
	char *m_category;

	char *m_name;
	char *m_desc;

	bool m_global;
	bool m_readOnly;

	//int m_minValue;
	//int m_maxValue;
	ExtendedAttributes *m_extendedAttributes;
	unsigned int m_numExtendedAttributes;

	char *m_purpose;
	char *m_notes;
};


class BHandHardware
{
public:
	BHandHardware(const char *modelNum, unsigned int numMotors, BHandProperty *listOfProperties, unsigned int numProperties) :
		m_numMotors(numMotors), m_propertyList(listOfProperties), m_numProperties(numProperties)
	{
		// Create a member copy of the Model Number
		m_modelNum = new char[strlen(modelNum) + 1];
		strcpy(m_modelNum, modelNum);

		// A pointer has been saved that points to the list of known properties
	}
	virtual ~BHandHardware()
	{
		delete[] m_modelNum;
	}

	// Accessor method for obtaining the model number
	const char * getModelNumber() { return m_modelNum; }


	///////////////////////////////////////////////////////////////////////////
	// Properties
	///////////////////////////////////////////////////////////////////////////

	// Accessor methods for obtaining known information on available properties
	BHandProperty * getBHandProperty(unsigned int propertyNum)
	{
		return (propertyNum < m_numProperties) ? &m_propertyList[propertyNum] : NULL;
	}
	unsigned int getBHandNumProperties() { return m_numProperties; }


	BHandProperty * getBHandProperty(const char *propertyName)
	{
		int p = getBHandPropertyNum(propertyName);
		return (p >= 0) ? getBHandProperty(p) : NULL;
	}

	int getBHandPropertyNum(const char *propertyName)
	{
		// Returns the property number from the name (or -1 if not found)
		for (unsigned int i = 0; i < m_numProperties; i++)
			if (strcmp(propertyName, m_propertyList[i].getName()) == 0)
				return i;

		return -1;
	}

	// Accessor method for obtaining the BarrettHand Hardware Description Index
	static int getBHandHardwareIndex(const char *modelNum)
	{
		// Search for the BarrettHand model number
		for (unsigned int i = 0; i < bhandGetHardwareNumModels(); i++)
		{
			BHandHardware *hw = bhandGetHardwareDesc(i);

			if (strcmp(hw->m_modelNum, modelNum) == 0)
				return i; // BarrettHand Hardware Description Found
		}

		return -1; // BarrettHand Hardware Description Not Found
	}

private:
	char *m_modelNum;

	unsigned int m_numMotors;

	BHandProperty *m_propertyList;
	unsigned int m_numProperties;
};


#endif
