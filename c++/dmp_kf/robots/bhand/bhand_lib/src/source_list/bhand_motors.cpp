#include "bhand_motors.h"

#include <ctype.h>
#include <string.h>


void toMotorChar(BHMotors bhMotors, char *chMotors)
{
	if (bhMotors != 15)
	{
		if ((bhMotors & 7) < 7)
		{
			if (bhMotors & 1) *(chMotors++) = '1';
			if (bhMotors & 2) *(chMotors++) = '2';
			if (bhMotors & 4) *(chMotors++) = '3';
		}
		else
			*(chMotors++) = 'G';
		if (bhMotors & 8) *(chMotors++) = 'S';
	}
	*chMotors = 0;
}

BHMotors toBHMotors(const char *motors)
{
	BHMotors bhmotors = 0;

	if (ContainsMotor(motors, '1')) bhmotors |= 1;
	if (ContainsMotor(motors, '2')) bhmotors |= 2;
	if (ContainsMotor(motors, '3')) bhmotors |= 4;
	if (ContainsMotor(motors, '4')) bhmotors |= 8;

	return bhmotors;
}

unsigned int countMotors(BHMotors bhMotors)
{
	unsigned int count = 0;
	while (bhMotors)
	{
		if (bhMotors & 1)
			count++;
		bhMotors >>= 1;
	}
	return count;
}

bool Contains(const char *str, const char ch)
{
	char c = tolower(ch);
	for (unsigned int i = 0; i < strlen(str); i++)
		if (tolower(str[i]) == c)
			return true;
	return false;
}

bool ContainsAllFingers(const char *motor)
{
	return (strlen(motor) == 0) || Contains(motor, 'g') || (Contains(motor, '1') & Contains(motor, '2') & Contains(motor, '3'));
}

bool ContainsAnyFingers(const char *motor)
{
	return (strlen(motor) == 0) || Contains(motor, 'g') || Contains(motor, '1') || Contains(motor, '2') || Contains(motor, '3');
}

bool ContainsSpread(const char *motor)
{
	return (strlen(motor) == 0) || Contains(motor, 's') || Contains(motor, '4');
}

bool ContainsMotor(const char *motorString, const char motorChar)
{
	char ch = tolower(motorChar);
	if (ch == 's')
		ch = '4';

	// Check that the passed in motor character represents a valid motor
	if ((ch >= '1' && ch <= '4') || ch == 's')
	{
		if (motorString[0] == 0)
			return true;

		// Look for just the motor character
		if (Contains(motorString, ch))
			return true;

		if (ch == '4' && Contains(motorString, 's'))
			return true;

		// Look to see if a motor character for a finger is contained if motorString contains 'g'
		if (Contains(motorString, 'g') && ch >= '1' && ch <= '3')
			return true;
	}

	return false;
}

/*void OptimizeMotorString(const char *motorString, char *optimizedMotorString)
{
	// optimizedMotorString will not be longer than motorString
	if (strlen(motorString) == 0)
		optimizedMotorString[0] = 0;
	else if (ContainsAllFingers(motorString))
	{
		if (ContainsSpread(motorString))
			optimizedMotorString[0] = 0;
		else
			optimizedMotorString[0] = 'G';
	}
	else
		strcpy(optimizedMotorString, motorString);
}*/
