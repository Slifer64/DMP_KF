#ifndef BHAND_MOTORS_H
#define BHAND_MOTORS_H


typedef int BHMotors;

// Helper functions to convert to new format that replaces motor strings
bool Contains(const char *str, const char ch);
bool ContainsAllFingers(const char *motor);
bool ContainsAnyFingers(const char *motor);
bool ContainsSpread(const char *motor);
bool ContainsMotor(const char *motorString, const char motorChar);

void toMotorChar(BHMotors bhMotors, char *chMotors);
BHMotors toBHMotors(const char *motors);

unsigned int countMotors(BHMotors bhMotors);

#endif
