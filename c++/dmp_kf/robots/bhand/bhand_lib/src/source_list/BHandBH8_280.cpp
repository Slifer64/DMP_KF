#include "BHandBH8_280.h"

#include "BHandHardware.h"


#define MIN_VALUE_INT32 -2147483648
#define MAX_VALUE_INT32 2147483647

#define MIN_VALUE_UINT32 0
#define MAX_VALUE_UINT32 4294967296


///////////////////////////////////////////////////////////////////////////////
// A list of all known properties for the BH8-280 BarrettHand
///////////////////////////////////////////////////////////////////////////////

BHandProperty propertyList_BH8_280[BHAND_BH8_280_NUM_PROPERTIES] = {

	// Motor properties (3, no S)
	BHandProperty("Motor",              "P",      "Present Position",                            true, r280finger, r280finger, r280finger, r280spread, "The present position of the motor.", "The range of the position parameter is dependent on the values of the OT (open target) and the CT (close target) parameters.  If these parameters are set beyond the joint stops then the joint stops themselves will dictate the range of position values, in which case the ranges may differ slightly from finger to finger."),
	//BHandProperty("Motor",              "OD",     "Odometer",                                   false,        r32,        r32,        r32,        r32, "The total number of counts traveled by the selected motor, divided by 1000.", "This value is never reset; it is maintained through power failures and firmware downloads."),
	BHandProperty("Motor",              "IOFF",     "Initialization Offset",                    false,        r32,        r32,        r32,        r32, "This is the distance this motor's origin is shifted away from the full open position.", "Adjusting this value on a finger motor also affects the force required to cause breakaway of the TorqueSwitchTM clutch.  Values further from zero result in less compression of the Belleville washer, and so result in lower breakaway forces; values closer to zero similarly result in higher breakaway forces."),

	BHandProperty("Motor",              "KP",       "PID Proportional Gain",                    false,        r16,        r16,        r16,        r16, "The proportional term makes a change to the motor output that is proportional to the present position error.", "This PID constant is used for both position and velocity mode."),
	BHandProperty("Motor",              "KI",       "PID Integral Gain",                        false,        r16,        r16,        r16,        r16, "The contribution from the integral term is proportional to both the magnitude and duration of the position error", "This PID constant is used for both position and velocity mode."),
	BHandProperty("Motor",              "KD",       "PID Derivative Gain",                      false,        r16,        r16,        r16,        r16, "The magnitude of the contribution of the derivative term (slope of the position error multiplied by KD) to the overall control action is termed the derivative gain.", "This PID constant is used for both position and velocity mode."),
	BHandProperty("Motor",              "STAT",     "Status",                                    true,       r0_2,       r0_2,       r0_2,       r0_2, "Puck 2 status: 0=Reset/Monitor, 2=Ready/Main", ""),

	// Movement properties (8)
	BHandProperty("Motion",           "ACCEL",    "Acceleration",                             false,       r16,       r16,       r16,       r16, "Maximum acceleration and deceleration (Q8.8 cts/ms/ms) when moving from one position to another.", "While the ACCEL parameter has a rather large range of values that it can accept, the motor can only follow a small subset of those values."),
	BHandProperty("Motion",           "CT",       "Close Target",                             false,r280finger,r280finger,r280finger,r280spread, "This is the position gone to by a C (\"Close\") command.", "The useful range of the CT property is bounded by the joint limits."),
	BHandProperty("Motion",           "DP",     "Default Position",                           false,r280finger,r280finger,r280finger,r280spread, "Destination of M command if no argument specified", "The approximate range of values due to joint limits is from -200,000 to 0 for the fingers and -36,000 to 0 for the spread."),
	BHandProperty("Motion",           "DS",     "Default Step",                               false,      r16s,      r16s,      r16s,      r16s, "Size of IC or IO command movement if no argument specified", "The step size DS has the range of a 16-bit integer from -32,768 to 32,767.  Positive values may be used but typically they are negative for the 280 hand."),
	BHandProperty("Motion",           "HOLD",     "Hold",                                     false,        r1,        r1,        r1,        r1, "If non-zero, then the motor is left energized after each motion command in order to hold the position constant.", "Since the fingers are not back-drivable, this is generally set to 1 only for the spread motor."),
	BHandProperty("Motion",           "IVEL",     "Initialization Velocity",                  false,  r16_4080,  r16_4080,  r16_4080,  r16_4080, "This value replaces MOV (\"Motor Open Velocity\") during initialization; this allows a consistent initialization velocity even if MOV is adjusted.", "Barrett Technology does not recommend increasing IVEL beyond its default value. Permanent deformation of Bellville washers could result. This would prevent the finger breakaway mechanism from engaging."),
	BHandProperty("Motion",           "MCV",    "Maximum Close Velocity",                     false,       r16,       r16,       r16,       r16, "Controls the maximum velocity (cts/ms) while closing a motor.", "Reads are presently redirected to read property MV.  Writes will write both MCV and MV."),
	BHandProperty("Motion",           "MOV",    "Maximum Open Velocity",                      false,       r16,       r16,       r16,       r16, "Controls the maximum velocity (cts/ms) while opening a motor.", "Reads are presently redirected to read property MV.  Writes will write both MOV and MV."),
	BHandProperty("Motion",           "OT",       "Open Target",                              false,r280finger,r280finger,r280finger,r280spread, "This is the position gone to by an O (\"Open\") command.", "The useful range of the OT property is bounded by the joint limits."),
	BHandProperty("Motion",           "TSTOP",    "Time to Stop",                             false,       r16,       r16,       r16,       r16, "Time in milliseconds before motor is considered stopped.", "WARNING: Please use caution when adjusting this parameter.  Setting TSTOP higher than its default can result in the motors heating up very quickly under moderate to heavy usage."),

	// RealTime properties (6, no LCVC, LCPG, LFVC, LFDP, or LFDPC)
	BHandProperty("RealTime",           "LCV",      "Loop Control Velocity",                    false, r1, r1, r1, r1, "If non-zero, then a velocity will be sent in the control block for the motor.", ""),
	BHandProperty("RealTime",           "LCT",      "Loop Control Torque",                      false, r1, r1, r1, r1, "If non-zero, then a torque will be sent in the control block for the motor.", ""),
	BHandProperty("RealTime",           "LCP",      "Loop Control Position",                    false, r1, r1, r1, r1, "If non-zero, then a position will be sent in the control block for the motor.", ""),

	BHandProperty("RealTime",           "LFV",      "Loop Feedback Velocity",                   false, r1, r1, r1, r1, "If non-zero, then the software reads the present velocity for the motor divided by the LFVC parameter.", ""),
	BHandProperty("RealTime",           "LFS",      "Loop Feedback Strain",                     false, r1, r1, r1, r1, "If non-zero, then the software reads the present strain gauge value for the motor.", ""),
	BHandProperty("RealTime",           "LFAP",     "Loop Feedback Absolute Position",          false, r1, r1, r1, r1, "If non-zero, then the software reads the present 32-bit position of the motor.", ""),
	BHandProperty("RealTime",           "LFPPS",    "Loop Feedback Pressure Profile Sensors",   false, r1, r1, r1, r1, "If non-zero, then the software reads the present pressure profile sensor readings.", ""),

	BHandProperty("RealTime",           "LFT",       "Loop Feedback Temperature",               false, r1, "If non-zero, then maximum of the present Puck temperatures is returned.", ""),

	// Strain Gauge properties (3, no HSG or LSG - check range)
	BHandProperty("Strain",             "HSG",    "Highest Strain Gauge Value",                 false,      r16,      r16,      r16,      r16, "Zero means, \"don't limit motor movement based on SG.\"", "If set to non-zero, the finger will move as required to limit the force seen at the gauge.  Not all hands have strain gauges installed."),
	BHandProperty("Strain",             "LSG",    "Lowest Strain Gauge Value",                  false,      r16,      r16,      r16,      r16, "Zero means, \"don't limit motor movement based on SG.\"", "If set to non-zero, the finger will move as required to limit the force seen at the gauge.  Not all hands have strain gauges installed."),
	BHandProperty("Strain",             "SG",     "Strain Gauge",                                true,      r12,      r12,      r12,      r12, "The present strain gauge value for the motor.", "Returns values between 0 to 4095."),

	// Advanced motor properties (6)
	//BHandProperty("Advanced - Motor",   "EN",       "Enabled",                                  false,        r1,        r1,        r1,        r1, "If non-zero, then a motion command with no motor prefix will act on this motor.", ""),
	//BHandProperty("Advanced - Motor",   "MPE",      "Maximum Position Error",                   false,       r16,       r16,       r16,       r16, "After moving to a desired position, if the position error is less than MPE then the move is considered a success.", "While MPE can be set as high as 65,535, its true range of useful values is bounded by the joint limits of the axes (e.g. approximately 0 to 18,000 for fingers and approximately 0 to 3150 for spread)."),
	//BHandProperty("Advanced - Motor",   "SGFLIP",   "Strain Gage Flip",                         false,        r1,        r1,        r1,        r1, "If non-zero, then all strain gage values for this motor are subtracted from 255 before being sent to the host.", "Used to invert strain gauge readings.  This is a legacy command and may not be supported in future releases.  It is strongly recommended that you avoid using SGFLIP since it may be dropped in future revisions of the firmware."),

	// Serial properties (1)
	BHandProperty("Serial",            "BAUD",      "Baud rate",                                false,  r6_384,  r6_384,  r6_384,  r6_384, "Controls the serial port baud rate in bits per second divided by 100.", ""),

	// Temperature properties (3)
	BHandProperty("Temperature",       "OTEMP",     "OverTemperature",                           true, r16, r16, r16, r16, "MT should be auto-reduced when TEMP equals OTEMP - 16.  At TEMP = OTEMP, MT should be set to zero", "Value is temperature in degrees Celsius."),
	BHandProperty("Temperature",       "TEMP",      "Temperature",                               true, r16, r16, r16, r16, "The present temperature on the Puck.", "The Units are in degrees Celsius."),
	//BHandProperty("Temperature",       "THERM",      "Thermistor (motor) Temperature",           true, r_550_1250, r_550_1250, r_550_1250, r_550_1250, "", ""),
	//BHandProperty("Temperature",       "PTEMP",     "Peak Temperature",                          true,    r0_1250, "The maximum temperature ever experienced by this hand", "This value is never reset; it is maintained through power failures and firmware downloads."),

	BHandProperty("Joint",              "JP",      "Inner Joint Position",                            true, r32, r32, r32, r32, "Gives the present inner link joint encoder position.", "Will return 0 if they are not installed."),

	// Info properties (3)
	//BHandProperty("Global",            "UPSECS",    "Uptime Seconds",                            true,        r32, "The total power-up time for this hand.", "This value is never reset; it is maintained through power failures and firmware downloads. The parameter can accommodate 136 years of power-up time before rolling over."),
	BHandProperty("Info",              "ID",        "Puck ID",                                   true,        r16,        r16,        r16,        r16, "The Puck identification number.", "Determines the 5-bit Puck ID that is used filter unwanted messages."),
	BHandProperty("Info",              "ROLE",      "Puck Role",                                 true,        r16,        r16,        r16,        r16, "The role of the Puck.", "Only use this property to check the value."),
	BHandProperty("Info",              "SN",        "Serial Number",                             true,        r16,        r16,        r16,        r16, "The serial number of the hand.", "This value is never reset; it is maintained through power failures and firmware downloads. Some pucks in the hand are not programmed with serial numbers."),
	BHandProperty("Info",              "VERS",      "Firmware Version",                          true,        r16,        r16,        r16,        r16, "The firmware version on the Puck.", "This value is never reset; it is maintained through power failures and firmware downloads.")

	};

