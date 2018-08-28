#ifndef BHAND_PARSE_H
#define BHAND_PARSE_H


#include "bhand_motors.h"

#include "puck2.h"


///////////////////////////////////////////////////////////////////////////////
// PUBLIC DEFINED constants
///////////////////////////////////////////////////////////////////////////////

// Define NONE as (N)egative (ONE), tee hee
#define NONE   (-1)

#define MAX_CMD_LEN (32)

#define MAX_COMMAND_RESULT_LENGTH 512


///////////////////////////////////////////////////////////////////////////////
// PUBLIC typedefs and structs
///////////////////////////////////////////////////////////////////////////////

typedef struct A_COMMAND
{
	char *ptr;
	char command[MAX_CMD_LEN];
} COMMAND;

#define MAX_PROP_LENGTH 15

typedef struct A_COMMAND_RESULT
{
	// parsed command input
	int command;        // parsed command
	BHMotors bhMotors;  // optional parsed motors included
	int p;              // optional parsed property
	bool valueIncluded; // optional parsed value included for commands such as step open/close
	long value;         // optional parsed value

	char propertyName[MAX_PROP_LENGTH + 1]; // pointer to property name

	// output - command result
	char *result;

} COMMAND_RESULT;

struct textKey
{
	char key[8];
	int     idx;
};

///////////////////////////////////////////////////////////////////////////////
// PUBLIC Methods
///////////////////////////////////////////////////////////////////////////////

int parseMessage(int id, int len, unsigned char *messageData,
                 int *node, int *property, long *value);

void parseInput(COMMAND *c, COMMAND_RESULT *cResult = 0);

int getPropertyValue(const char *propertyText);
int getPropertyValue(const char *propertyText, const struct textKey *t);


#endif
