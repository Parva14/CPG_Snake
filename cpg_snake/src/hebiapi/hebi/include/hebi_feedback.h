/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_FEEDBACK_H
#define HEBI_FEEDBACK_H

#include "stdint.h"
#include "hebi_vector_3_f.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiFeedback* HebiFeedbackPtr;

// Define all fields
typedef enum FeedbackFloatField {
FeedbackFloatBoardTemperature, ///Ambient temperature inside the module (measured at the IMU chip), in degrees Celsius.
FeedbackFloatProcessorTemperature, ///Temperature of the processor chip, in degrees Celsius.
FeedbackFloatVoltage, ///Bus voltage that the module is running at (in Volts).
FeedbackFloatVelocity, ///Velocity of the module output (post-spring), in radians/second.
FeedbackFloatTorque, ///Torque at the module output, in N * m.
FeedbackFloatVelocityCommand, ///Commanded velocity of the module output (post-spring), in radians/second.
FeedbackFloatTorqueCommand, ///Commanded torque at the module output, in N * m.
FeedbackFloatDeflection, ///Difference (in radians) between the pre-spring and post-spring output position.
FeedbackFloatDeflectionVelocity, ///Velocity (in radians/second) of the difference between the pre-spring and post-spring output position.
FeedbackFloatMotorVelocity, ///The velocity (in radians/second) of the motor shaft.
FeedbackFloatMotorCurrent, ///Current supplied to the motor.
FeedbackFloatMotorSensorTemperature, ///The temperature from a sensor near the motor housing.
FeedbackFloatMotorWindingCurrent, ///The estimated current in the motor windings.
FeedbackFloatMotorWindingTemperature, ///The estimated temperature of the motor windings.
FeedbackFloatMotorHousingTemperature, ///The estimated temperature of the motor housing.
} FeedbackFloatField;

typedef enum FeedbackHighResAngleField {
FeedbackHighResAnglePosition, ///Position of the module output (post-spring), in radians.
FeedbackHighResAnglePositionCommand, ///Commanded position of the module output (post-spring), in radians.
} FeedbackHighResAngleField;

typedef enum FeedbackNumberedFloatField {
FeedbackNumberedFloatDebug, ///Values for internal debug functions (channel 1-9 available).
} FeedbackNumberedFloatField;

typedef enum FeedbackVector3fField {
FeedbackVector3fAccelerometer, ///Accelerometer data, in m/s^2.
FeedbackVector3fGyro, ///Gyro data, in radians/second.
} FeedbackVector3fField;

typedef enum FeedbackIoPinBank {
FeedbackIoBankA, ///I/O pin bank a (pins 1-8 available)
FeedbackIoBankB, ///I/O pin bank b (pins 1-8 available)
FeedbackIoBankC, ///I/O pin bank c (pins 1-8 available)
FeedbackIoBankD, ///I/O pin bank d (pins 1-8 available)
FeedbackIoBankE, ///I/O pin bank e (pins 1-8 available)
FeedbackIoBankF, ///I/O pin bank f (pins 1-8 available)
} FeedbackIoPinBank;

typedef enum FeedbackLedField {
FeedbackLedLed, ///The module's LED.
} FeedbackLedField;

/**
 * Returns value for given field. If HasFloat() returns 0, this results in
 * undefined behavior.
 */
float hebiFeedbackGetFloat(HebiFeedbackPtr, FeedbackFloatField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiFeedbackHasFloat(HebiFeedbackPtr, FeedbackFloatField);

/**
 * Returns value for given field. If HasHighResAngle() returns 0, this results in
 * undefined behavior.
 */
void hebiFeedbackGetHighResAngle(HebiFeedbackPtr, FeedbackHighResAngleField, int64_t* int_part, float* dec_part);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiFeedbackHasHighResAngle(HebiFeedbackPtr, FeedbackHighResAngleField);

/**
 * Returns value for given field. If HasNumberedFloat() for this number returns
 * 0, this results in undefined behavior.
 */
float hebiFeedbackGetNumberedFloat(HebiFeedbackPtr, FeedbackNumberedFloatField, int);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiFeedbackHasNumberedFloat(HebiFeedbackPtr, FeedbackNumberedFloatField, int);

/**
 * Returns value for given field. If HasVector3f() returns 0, this results in
 * undefined behavior.
 */
HebiVector3f hebiFeedbackGetVector3f(HebiFeedbackPtr, FeedbackVector3fField);
/**
 * Indicates whether the specified value is set; returns '1' for yes and '0' for
 * no.
 */
int hebiFeedbackHasVector3f(HebiFeedbackPtr, FeedbackVector3fField);

/**
 * Gets the integer value of a given pin, if it exists.
 */
int64_t hebiFeedbackGetIoPinInt(HebiFeedbackPtr, FeedbackIoPinBank, unsigned int pin_number);
/**
 * Gets the floating point value of a given pin, if it exists
 */
float hebiFeedbackGetIoPinFloat(HebiFeedbackPtr, FeedbackIoPinBank, unsigned int pin_number);
/**
 * Indicates whether the given pin has an integer value; returns '1' for yes and
 * '0' for no.
 */
int hebiFeedbackHasIoPinInt(HebiFeedbackPtr, FeedbackIoPinBank, unsigned int pin_number);
/**
 * Indicates whether the given pin has an floating point value; returns '1' for yes and
 * '0' for no.
 */
int hebiFeedbackHasIoPinFloat(HebiFeedbackPtr, FeedbackIoPinBank, unsigned int pin_number);

/**
 * Returns led color into the three output integers (each 0-255). If HasLedColor() returns 0, this results in
 * undefined behavior.
 */
void hebiFeedbackGetLedColor(HebiFeedbackPtr, FeedbackLedField, uint8_t *r, uint8_t *g, uint8_t *b);
/**
 * Indicates whether the led color is set; returns '1' for yes and '0' for no. For command-style messages, this refers
 * to a command to override the module's default control of the LED.
 */
int hebiFeedbackHasLedColor(HebiFeedbackPtr, FeedbackLedField);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_FEEDBACK_H
