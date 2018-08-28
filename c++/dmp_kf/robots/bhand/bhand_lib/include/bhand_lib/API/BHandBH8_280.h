#ifndef BHAND_BH8_280_H
#define BHAND_BH8_280_H

// Forward declaration for a BarrettHand property
class BHandProperty;

// Exposed hardware constants for the BH8-280 BarrettHand
const char BHAND_BH8_280_MODEL_NUM[] = "BH8-280";
const unsigned int BHAND_BH8_280_NUM_PROPERTIES = 35;
const int BHAND_BH8_280_NUM_MOTORS = 4;

extern BHandProperty propertyList_BH8_280[BHAND_BH8_280_NUM_PROPERTIES];

#endif
