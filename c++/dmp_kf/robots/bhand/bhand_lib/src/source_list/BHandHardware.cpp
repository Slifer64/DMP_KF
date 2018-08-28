#include "BHandHardware.h"

// Include header files for different BarrettHand models
#ifdef BH8_262_HARDWARE
	#include "BHandBH8_262.h"
#endif
#ifdef BH8_280_HARDWARE
	#include "BHandBH8_280.h"
#endif

// Variables for BarrettHand Hardware Descriptions
unsigned int bhandNumModels = 0;
BHandHardware **bhandHardware = NULL;


void bhandCreateHardwareDesc()
{
	// Do a quick check that hardware has not been created
	if (bhandHardware != NULL)
		return;

	// Keep track of the number of BarrettHand models
	bhandNumModels = 0;

#ifdef BH8_262_HARDWARE
	bhandNumModels++;
#endif
#ifdef BH8_280_HARDWARE
	bhandNumModels++;
#endif

	// Allocate memory on the heap for BarrettHands
	bhandHardware = new BHandHardware * [bhandNumModels];

	int modelCount = 0;

	///////////////////////////////////////////////////////////////////////////
	// Create Hardware descriptions for BarrettHands
	///////////////////////////////////////////////////////////////////////////

#ifdef BH8_262_HARDWARE
	bhandHardware[modelCount++] = new BHandHardware(
			BHAND_BH8_262_MODEL_NUM,
			BHAND_BH8_262_NUM_MOTORS,
			propertyList_BH8_262, BHAND_BH8_262_NUM_PROPERTIES);
#endif

#ifdef BH8_280_HARDWARE
	bhandHardware[modelCount++] = new BHandHardware(
			BHAND_BH8_280_MODEL_NUM,
			BHAND_BH8_280_NUM_MOTORS,
			propertyList_BH8_280, BHAND_BH8_280_NUM_PROPERTIES);
#endif

}

void bhandDestroyHardwareDesc()
{
	// Do a quick check that hardware has not been created
	if (bhandHardware == NULL)
		return;

	// Delete BarrettHand Hardware Descriptions
	for (unsigned int i = 0; i < bhandNumModels; i++)
		delete bhandHardware[i];

	delete[] bhandHardware;

	bhandHardware = NULL;
}

unsigned int bhandGetHardwareNumModels() { return bhandNumModels; }
BHandHardware * bhandGetHardwareDesc(unsigned int hwIndex) { return (hwIndex < bhandNumModels) ? bhandHardware[hwIndex] : NULL; }
