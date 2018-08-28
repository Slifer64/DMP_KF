#ifndef PUCKSINHAND_H
#define PUCKSINHAND_H

//#include "PucksInHandSupervisory.h"
//#include "PucksInHandRealTime.h"
#include "PucksInHandCommon.h"

///////////////////////////////////////////////////////////////////////////////
// Accessor methods
///////////////////////////////////////////////////////////////////////////////

int pucksInHandPropertyNumDefault();
int pucksInHandPropertyNumSpreadDefault();

void pucksInHandPropertyDefault(int index, int *property, int *value);
void pucksInHandPropertySpreadDefault(int index, int *property, int *value);


#endif
