#include "bhand_parse.h"

#include "puck2.h"

#include <string.h>

#include <stdio.h>


#define ADDR2NODE(x) ((((x) >> 5) & 0x001F) - BASE_ID)
#define BASE_ID (0)


///////////////////////////////////////////////////////////////////////////////
// PRIVATE MACRO definitions
///////////////////////////////////////////////////////////////////////////////

// Define character identification macros
#define isAlpha(c) (((c) >= 'A') && ((c) <= 'Z'))
#define isDigit(c) (((c) >= '0') && ((c) <= '9'))
#define isSpace(c) ( (c) == SP )
#define toUpper(c) ( ((c >= 'a') && ((c) <= 'z')) ? ((c) - 'a' + 'A') : (c) )


// Command text. Keyword, enumeration. Add alphabetically.
const struct textKey cmdTxt[]=
	{
		{ "FLOAD", CMD_LOAD },
		{ "FSAVE", CMD_SAVE },
		{ "RESET", CMD_RESET },
		{ "FDEF", CMD_DEF },
		{ "FGET", CMD_GET },
		{ "FIND", CMD_FIND },
		{ "FSET", CMD_SET },
		{ "HOME", CMD_HOME },
		{ "KEEP", CMD_KEEP },
		{ "LOAD", CMD_LOAD },
		{ "LOOP", CMD_LOOP },
		{ "PASS", CMD_PASS },
		{ "SAVE", CMD_SAVE },
		{ "VERS", CMD_VERS },
		{ "DEF", CMD_DEF },
		{ "ERR", CMD_ERR },
		{ "GET", CMD_GET },
		{ "SET", CMD_SET },
		{ "HI", CMD_HI },
		{ "IC", CMD_IC },
		{ "IO", CMD_IO },
		{ "TC", CMD_TC },
		{ "TO", CMD_TO },
		{ "C", CMD_C },
		{ "M", CMD_M },
		{ "O", CMD_O },
		{ "T", CMD_T },
		{ "?", CMD_HELP },

		{ "", NONE }
	};


const struct textKey propTxtCommon[] =
	{
		{ "ILOGIC", PROP_ILOGIC }, // Logic current (tbd)
		{ "IMOTOR", PROP_IMOTOR }, // Motor current (2048+205/1A)
		{ "VLOGIC", PROP_VLOGIC }, // Logic voltage (tbd)
		{ "ERROR", PROP_ERROR }, // Error (tbd)
		{ "OTEMP", PROP_OTEMP }, // Over temperature alarm (tbd)
		{ "PTEMP", PROP_PTEMP }, // Peak temperature recorded (tbd)
		{ "THERM", PROP_THERM }, // Thermistor (motor) temperature
		{ "VALUE", PROP_VALUE }, // Value to poke/peek
		{ "ADDR", PROP_ADDR }, // Address to peek/poke
		{ "ANA0", PROP_ANA0 }, // Analog input (pin 4:0-3V, 3:Gnd)
		{ "ANA1", PROP_ANA1 }, // Analog input (pin 2:0-3V, 3:Gnd)
		{ "BAUD", PROP_BAUD }, // Baud rate/100. 96=9600 bps
		{ "DIG0", PROP_DIG0 }, // Dig I/O: -1=In,0=Lo,1=Hi,2-100=%PWM (pin 41:0-3.3V, 44:Gnd)
		{ "DIG1", PROP_DIG1 }, // Dig I/O: -1=In,0=Lo,1=Hi (pin 43:0-3.3V, 44:Gnd)
		{ "FET0", PROP_FET0 }, // Tensioner output: 0=Off, 1=On
		{ "FET1", PROP_FET1 }, // Brake output: 0=Off, 1=On
		{ "FIND", PROP_FIND }, // Find command for CAN
		{ "GRPA", PROP_GRPA }, // Comm group A
		{ "GRPB", PROP_GRPB }, // Comm group B
		{ "GRPC", PROP_GRPC }, // Comm group C
		{ "LOAD", PROP_LOAD }, // Load command for CAN
		{ "LOCK", PROP_LOCK }, // Lock
		{ "MODE", PROP_MODE }, // Mode: 0=Idle, 2=Torque, 3=PID, 4=Vel, 5=Trap
		{ "ROLE", PROP_ROLE }, // P=PRODUCT, R=ROLE: XXXX PPPP XXXX RRRR
		{ "SAVE", PROP_SAVE }, // Save command for CAN
		{ "STAT", PROP_STAT }, // Status: 0=Reset/Monitor, 2=Ready/Main
		{ "TEMP", PROP_TEMP }, // Temperature (puck internal)
		{ "VBUS", PROP_VBUS }, // Bus voltage (V)
		{ "VERS", PROP_VERS }, // Firmware version
		{ "CMD", PROP_CMD }, // For commands w/o values: RESET,HOME,KEEP,PASS,LOOP,HI,IC,IO,TC,TO,C,O,T
		{ "DEF", PROP_DEF }, // Default command for CAN
		{ "ID", PROP_ID }, // CANbus ID
		{ "SG", PROP_SG }, // Strain gage (tbd)
		{ "SN", PROP_SN }, // Serial number
		{ "X0", PROP_X0 }, // Gimbals offset 1 (Q4.12 rad)
		{ "X1", PROP_X1 }, // Gimbals offset 2 (Q4.12 rad)
		{ "X2", PROP_X2 }, // Gimbals offset 3 (Q4.12 rad)
		{ "X3", PROP_X3 }, // Gimbals/safety gain 1 (Q4.12 rad/3V)
		{ "X4", PROP_X4 }, // Gimbals/safety gain 2 (Q4.12 rad/3V)
		{ "X5", PROP_X5 }, // Gimbals/safety gain 3 (Q4.12 rad/3V)
		{ "X6", PROP_X6 }, // tbd
		{ "X7", PROP_X7 }, // tbd

		{ "", NONE }
	};


const struct textKey propTxtTater[]=
    {
        { "LFLAGS", PROP_LFLAGS }, /* Loop feedback flags */
        { "UPSECS", PROP_UPSECS }, /* Up seconds in operation (tbd) */
        { "ACCEL", PROP_ACCEL }, /* Acceleration (Q8.8 cts/ms/ms) */
        { "ECMAX", PROP_ECMAX }, /* Encoder correction max value */
        { "ECMIN", PROP_ECMIN }, /* Encoder correction min value */
        { "HALLH", PROP_HALLH }, /* 32-Bit Hall history bitfield */
        { "HALLS", PROP_HALLS }, /* Hall feedback bitfield: CBA */
        { "IKCOR", PROP_IKCOR }, /* Current sense correction factor */
        { "IOFST", PROP_IOFST }, /* Current offset calibration */
        { "JOFST", PROP_JOFST }, /* Joint encoder calibration offset */
        { "MOFST", PROP_MOFST }, /* Mechanical offset calibration */
        { "POLES", PROP_POLES }, /* Number of magnets on rotor */
        { "TENSO", PROP_TENSO }, /* Tension offset (tbd) */
        { "TENST", PROP_TENST }, /* Tension total (tbd) */
        { "TSTOP", PROP_TSTOP }, /* Time until considered stopped */
        { "HOLD", PROP_HOLD }, /* Flag to hold position after move */
        { "IOFF", PROP_IOFF }, /* Initialization offset */
        { "IPNM", PROP_IPNM }, /* CommandedCurrent / Nm (ratio) */
        { "IVEL", PROP_IVEL }, /* Initialization velocity (tbd) */
        { "JIDX", PROP_JIDX }, /* Joint index */
        { "LCTC", PROP_LCTC }, /* Loop control torque coefficient */
        { "LCVC", PROP_LCVC }, /* Loop control velocity coefficient */
        { "MECH", PROP_MECH }, /* 32-Bit Mechanical angle (cts) */
        { "PIDX", PROP_PIDX }, /* Puck index for torque */
        { "CTS", PROP_CTS }, /* 32-Bit Counts per revolution */
        { "HSG", PROP_HSG }, /* High strain gage (tbd) */
        { "IKI", PROP_IKI }, /* Current sense integral gain */
        { "IKP", PROP_IKP }, /* Current sense proportional gain */
        { "LSG", PROP_LSG }, /* Low strain gage (tbd) */
        { "MCV", PROP_MCV }, /* Max close velocity (cts/ms) */
        { "MDS", PROP_MDS }, /* Max duty sum for power limiting (tbd) */
        { "MOV", PROP_MOV }, /* Max open velocity (cts/ms) */
        { "MPE", PROP_MPE }, /* Max position error (tbd) */
        { "TIE", PROP_TIE }, /* Flag to tie inner and outer links */
        { "CT", PROP_CT }, /* 32-Bit Close Target */
        { "DP", PROP_DP }, /* 32-Bit Default Position */
        { "DS", PROP_DS }, /* Default step */
        { "EN", PROP_EN }, /* Enable bitfield */
        { "JP", PROP_JP }, /* Joint encoder position */
        { "KD", PROP_KD }, /* Differential gain */
        { "KI", PROP_KI }, /* Integral gain */
        { "KP", PROP_KP }, /* Proportional gain */
        { "MT", PROP_MT }, /* Max torque */
        { "MV", PROP_MV }, /* Max velocity (cts/ms) */
        { "OD", PROP_OD }, /* Odometer (tbd) */
        { "OT", PROP_OT }, /* 32-Bit Open Target */
        { "E", PROP_E }, /* 32-Bit Endpoint */
        { "M", PROP_M }, /* 32-Bit Move command for CAN */
        { "P", PROP_P }, /* 32-Bit Position. R=Act, W=Cmd */
        { "T", PROP_T }, /* Torque command */
        { "V", PROP_V }, /* Velocity (cts/ms).  R=Act, W=Cmd */

        { "", NONE }
    };


void uppercase(char *s)
{
	//while (*s++ = toUpper(*s));
	while (*s)
	{
		if (*s >= 'a' && *s <= 'z')
			*s -= 'a' - 'A';
		s++;
	}
}





///////////////////////////////////////////////////////////////////////////
// Methods for parsing
///////////////////////////////////////////////////////////////////////////



int parseMessage(
	// Input
	int id, int len, unsigned char *messageData,

	// Output
	int *node, int *property, long *value)

	// Input
	// id - The message ID
	// len - The data payload length
	// messageData - Pointer to the message data payload

	// Output
	// node - The controller node ID of the received message
	// property - The property this message applies to
	// value - The value of the property being processed

{
	int i;
	int dataHeader;

	*node = ADDR2NODE(id);

	//if (*node == -1)
	//   syslog(LOG_ERR,"msgID:%x ",id);

	// Sets dataHeader to 2 when high bit is set and dataHeader to 1 if id is from 2 ('00010') to 3 ('00011')
	dataHeader = ((messageData[0] >> 6) & 0x0002); // | ((id & 0x041F) == 0x0403);

	//messageData[0] &= 0x7F;
	//syslog(LOG_ERR,"Entering parsemessage");

	switch (dataHeader)
	{
		/*case 3:  // Data is a packed 22-bit position, SET
			*value = 0x00000000;
			*value |= ( (long)messageData[0] << 16) & 0x003F0000;
			*value |= ( (long)messageData[1] << 8 ) & 0x0000FF00;
			*value |= ( (long)messageData[2] ) & 0x000000FF;

			if (*value & 0x00200000) // If negative
				*value |= 0xFFC00000; // Sign-extend

			*property = PROP_P;

			// TODO: these were commented out
			//jointPosition[*node] = 0;
			//jointPosition[*node] |= ( (long)messageData[3] << 16) & 0x003F0000;
			//jointPosition[*node] |= ( (long)messageData[4] << 8 ) & 0x0000FF00;
			//jointPosition[*node] |= ( (long)messageData[5] ) & 0x000000FF;

			//if (jointPosition[*node] & 0x00200000) // If negative
			//   jointPosition[*node] |= 0xFFC00000; // Sign-extend

			//syslog(LOG_ERR,"Received packed set property: %d from node: %d value:%d",*property,*node,*value);
			break;*/
		case 2:  // Data is normal, SET
			*property = messageData[0] & 0x7F;
			//syslog(LOG_ERR, "Received property: %d", *property);
			/* Store the value, second byte of message is zero (for DSP word alignment) */
			*value = messageData[len-1] & 0x80 ? -1L : 0;
			for (i = len-1; i >= 2; i--)
				*value = *value << 8 | messageData[i];

			//syslog(LOG_ERR, "Received normal set property: %d from node: %d value:%d", *property, *node, *value);
			//syslog(LOG_ERR,"parsemessage after %d",value);
			break;
		case 0:  // Assume firmware request (GET)
			*property = -(messageData[0] & 0x7F); // A negative (or zero) property means GET
			*value = 0;
			//syslog(LOG_ERR, "Received normal get property: %d from node: %d value:%d", *property, *node, *value);

			break;
		default:
		{
			//syslog(LOG_ERR, "<Illegal Message Header> %d\n", dataHeader);
			return(1);
		}
	}

	//if (*property != 8) syslog(LOG_ERR,"Value in parsemessage is: %d",*value);
	return (0);
}


int getPropertyValue(const char *propertyText, const struct textKey *t)
{
	int i = 0;

	while (*t[i].key) // "key is not NULL"
	{
		if (strstr(propertyText, t[i].key) == propertyText) // Property matches, followed by...
		{
			if (*(propertyText + strlen(t[i].key)) <= ' ') // ...space (32) or null (0)
				return t[i].idx; // Property text found
		}
		++i;
	}

	return NONE;
}

int getPropertyValue(const char *propertyText)
{
	int p = getPropertyValue(propertyText, propTxtCommon); // Check for common prop
	if (p == NONE)
		p = getPropertyValue(propertyText, propTxtTater); // Check for tater prop
	return p;
}


// Determine the property (for GET FGET SET FSET)
inline int parseText(COMMAND *c, const struct textKey *t)
{
	int i;

	i = 0;
	while (*t[i].key)
	{
		if (strstr(c->ptr, t[i].key) == c->ptr) // Command matches, followed by...
			if (*(c->ptr + strlen(t[i].key)) <= ' ') // ...space (32) or null (0)
				break; // Found the command
		++i;
	}
	c->ptr += strlen(t[i].key);

	return (t[i].idx);
}


// Convert [1234567GS][IL][OL] into a Wraptor motor select bitfield
long parseMotorSelect(COMMAND *c)
{
	long motorSelect; // XXXXXXXX76543210

	motorSelect = 0;

	while (1)
	{
		if (isDigit(*c->ptr))
		{
			motorSelect |= 1L << (*c->ptr - '0');
			++c->ptr;

			continue;
		}

		switch (*c->ptr)
		{
			case ('S'):
				// Must NOT be followed by 'E' (for SET)
				if (*(c->ptr+1) == 'E')
					return(motorSelect);
				// Must NOT be followed by 'AV' (for SAVE)
				if ((*(c->ptr+1) == 'A') && (*(c->ptr+2) == 'V'))
					return(motorSelect);
				motorSelect |= 0x0010; // Motor 4 = 00010000
				++c->ptr;

				break;
			case ('G'):
				// Must NOT be followed by 'E' (for GET)
				if(*(c->ptr+1) == 'E')
					return (motorSelect);
				motorSelect |= 0x000E; // 1, 2, 3  = 00001110
				++c->ptr;

				break;
			case ('I'):
				// Must be followed by 'L' (for Inner Link)
				if(*(c->ptr+1) != 'L')
					return(motorSelect);
				motorSelect |= 0x000E; // 1, 2, 3 = 00001110
				c->ptr += 2;

				break;
			case ('O'): // The letter oh
				// Must be followed by 'L' (for Outer Link)
				if(*(c->ptr+1) != 'L')
					return(motorSelect);
				motorSelect |= 0x00E0; // 5, 6, 7 = 11100000
				c->ptr += 2;

				break;
			default:
				return(motorSelect);
		}
	}
}




// Converts a string to a long
// Return 0 for successful conversion, 1 for no conversion

int parse_atol(char *s, long *v)
// s - Pointer to string to convert to integer
// v - Pointer to location to put the value
{
	int i = 0;   // counter
	short sign;  // is there a sign in the string
	long addval; // the value of the character currently adding
	long result; // the result of the function

	result = 0;

	if ((sign = (*s == '-')) || *s == '+')
		i++;

	while (*(s+i)>='0' && *(s+i)<='9')
	{
		addval = *(s+i) - '0';

		if (result == (addval = (result * 10 + addval)) / 10)
			result = addval;

		++i;
	}

	if (sign)
		result = -result;

	*v = result;

	return (!i);
}


void parseInput(COMMAND *c, COMMAND_RESULT *cResult)
{
	int err;
	long motorSelect;

	// Set default value
	cResult->value = 0;

	// Uppercase the (null terminated) input string
	uppercase(c->command);

	// Set ptr to point to the beginning of the command string to "send"
	c->ptr = c->command;

	// Skip WhiteSpace
	while (*c->ptr == ' ')
		++c->ptr;

	// Determine the motor select
	// 0 1 2 3 4 5 6 7 G S IL (InnerLinks) OL (OuterLinks)
	motorSelect = parseMotorSelect(c);

	//printf("parseInput motorSelect = %d\n", motorSelect);

	// motorSelect will contain bits set (bits 1 - 4 for selected motors)

	// Check to see if no motors are selected (select all of them if none are selected)
	if (!motorSelect)
	{
		// If no motors specified (leave bit 0 blank for motor 0 bit - not used)
		motorSelect = 0x001E; // Specify all: 00011110
	}

	///////////////////////////////////////////////////////////////////////////
	// Create BHMotors bitfield containing selected motors
	///////////////////////////////////////////////////////////////////////////

	cResult->bhMotors = ((unsigned int)motorSelect & 0x1f) >> 1;

	//printf("parseInput cResult->bhMotors = %d\n", cResult->bhMotors);


	// Skip WhiteSpace
	while (*c->ptr == ' ')
		++c->ptr;


	///////////////////////////////////////////////////////////////////////////
	// Determine the command
	///////////////////////////////////////////////////////////////////////////

	cResult->command = parseText(c, cmdTxt);

	// Commands may be:
	//	RESET FSET SET FGET GET FLOAD LOAD FSAVE SAVE FDEF DEF
	//	HOME IO IC HI VERS ERR O C M T ?

	// If no command specified
	if (cResult->command == NONE)
		return;

	// Skip WhiteSpace
	while (*c->ptr == ' ')
		++c->ptr;






	///////////////////////////////////////////////////////////////////////////
	// New code for parsing properties/values
	///////////////////////////////////////////////////////////////////////////

	cResult->valueIncluded = false;

	char *str = c->ptr;//"- This, a sample string.";
	char *pch;
	//printf("Splitting string \"%s\" into tokens:\n",str);
	pch = strtok(str," ");
	while (pch != NULL)
	{
		//printf("Found token: %s\n",pch);

		if (cResult->command == CMD_SET ||
		    cResult->command == CMD_GET ||
		    cResult->command == CMD_FIND ||
		    cResult->command == CMD_LOAD ||
		    cResult->command == CMD_SAVE ||
		    cResult->command == CMD_DEF)
		{

				//printf("  parseInput determine property cases\n");

				///////////////////////////////////////////////////////////////
				// Determine property by token
				///////////////////////////////////////////////////////////////

				c->ptr = pch;
				if (strlen(pch) < MAX_PROP_LENGTH)
					strcpy(cResult->propertyName, pch);
				else
					cResult->propertyName[0] = 0; // No property name
				//printf("  parseInput propertyName token = %s\n", cResult->propertyName);

				///////////////////////////////////////////////////////////////
				// Determine Puck2 property by number
				///////////////////////////////////////////////////////////////

				// New

				cResult->p = parseText(c, propTxtCommon);
				if (cResult->p == NONE) // If no common property specified
				{
					// Check for property in TxtTater
					cResult->p = parseText(c, propTxtTater);

					// property will be set to NONE if not in propTxtTater
				}

				//printf("  parseInput cResult->p = %d\n", cResult->p);


				// Skip WhiteSpace
				//while(*c->ptr == ' ')
				//	++c->ptr;

				pch = strtok(NULL, " ");
				c->ptr = pch;
				//printf("Found token: %s\n", pch);

		}
		else
		{
			cResult->p = NONE;
		}


		///////////////////////////////////////////////////////////////////////
		// Determine possible values that may appear as tokens
		///////////////////////////////////////////////////////////////////////

		//cResult->valueIncluded = false;

		//	CMD_LOAD, CMD_SAVE,	CMD_RESET, CMD_DEF, CMD_GET, CMD_FIND, CMD_SET, CMD_HOME,
		//	CMD_KEEP, CMD_LOOP, CMD_PASS, CMD_VERS, CMD_ERR, CMD_HI, CMD_IC, CMD_IO,
		//	CMD_TC, CMD_TO, CMD_C, CMD_M, CMD_O, CMD_T, CMD_HELP, CMD_END

		switch (cResult->command)
		{
			case CMD_IC:  // "IC"
			case CMD_IO:  // "IO"
			case CMD_SET: // "SET"
				{
					// Determine the value
					err = parse_atol(c->ptr, &(cResult->value));

					// Mark value as being included
					cResult->valueIncluded = !err;

					break;
				}
			case CMD_M: // "M"
				{
					// Determine the value
					err = parse_atol(c->ptr, &(cResult->value));
					if (!err)
					{
						// If we have a value, just do a "SET M"
						//cResult->command = CMD_SET;
						//cResult->p = PROP_M;

						// Mark value as being included
						cResult->valueIncluded = true;
					}
					break;
				}
		}

		//if (cResult->valueIncluded)
		//{
		//	printf("  parseInput cResult->value = %d\n", cResult->value);
		//	printf("  parseInput valueIncluded = true\n");
		//}
		//else
		//	printf("  parseInput valueIncluded = false\n");

		break; // Don't care about any more tokens that may be found
	}





	///////////////////////////////////////////////////////////////////////////
	// Determine the property
	///////////////////////////////////////////////////////////////////////////
/*
	printf("parseInput determine property\n");

	switch (cResult->command)
	{
		case CMD_SET: // "SET"
		case CMD_GET: // "GET"
		case CMD_FIND: // "FIND"
		case CMD_LOAD: // "LOAD"
		case CMD_SAVE: // "SAVE"
		case CMD_DEF:  // "DEF"
			{
				printf("parseInput determine property cases\n");

				cResult->p = parseText(c, propTxtCommon);
				if (cResult->p == NONE) // If no common property specified
				{
					// Check for property in TxtTater
					cResult->p = parseText(c, propTxtTater);

					// property will be set to NONE if not in propTxtTater
				}

				printf("parseInput cResult->p = %d\n", cResult->p);

				// Skip WhiteSpace
				while(*c->ptr == ' ')
					++c->ptr;

				break;
			}
		default:
			{
				cResult->p = NONE;
			}
	}


	///////////////////////////////////////////////////////////////////////////
	// Read a value for certain commands
	//
	// parse_atol - Converts a string to a long
	//              Return 0 for successful conversion, 1 for no conversion
	//
	///////////////////////////////////////////////////////////////////////////

	cResult->valueIncluded = false;

//	CMD_LOAD, CMD_SAVE,	CMD_RESET, CMD_DEF, CMD_GET, CMD_FIND, CMD_SET, CMD_HOME,
//	CMD_KEEP, CMD_LOOP, CMD_PASS, CMD_VERS, CMD_ERR, CMD_HI, CMD_IC, CMD_IO,
//	CMD_TC, CMD_TO, CMD_C, CMD_M, CMD_O, CMD_T, CMD_HELP, CMD_END

    switch (cResult->command)
    {
    	case CMD_IC:  // "IC"
    	case CMD_IO:  // "IO"
		case CMD_SET: // "SET"
			{
				// Determine the value
				err = parse_atol(c->ptr, &(cResult->value));

				// Mark value as being included
				cResult->valueIncluded = !err;

				break;
			}
		case CMD_M: // "M"
			{
				// Determine the value
				err = parse_atol(c->ptr, &(cResult->value));
				if (!err)
				{
					// If we have a value, just do a "SET M"
					cResult->command = CMD_SET;
					cResult->p = PROP_M;

					// Mark value as being included
					cResult->valueIncluded = true;
				}
				break;
			}
    }*/
}



///////////////////////////////////////////////////////////////////////////////
// toMatrix - Allocates space for a string representation of the matrix
//            and returns a pointer to that matrix if successful.
///////////////////////////////////////////////////////////////////////////////

char * toMatrix(const int **values, unsigned int m, unsigned int n)
{
	// Check for no input
	if (m == 0 || n == 0)
		return NULL;

	// Allocate space greater than:
	//    11 characters per value (e.g -2000000000)
	//    + 1 character for a space
	//    + 2 characters for '\r' and '\n'
	//    + 1 character for NULL termination character
	char *matrix = new char[(m * n) * 16];

	char sval[16];

	for (unsigned int i = 0; i < n; i++)
	{
		for (unsigned int j = 0; j < m; j++)
		{
			if (i == 0)
				sprintf(sval, "%d", values[m][n]); // or should it be values[n][m]
			else
				sprintf(sval, " %d", values[m][n]); // or should it be values[n][m]

			strcat(matrix, sval);
		}
		if (i != n - 1)
			strcat(matrix, "\r\n");
	}
	return matrix;
}
