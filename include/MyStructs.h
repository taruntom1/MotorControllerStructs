// In MyStructs.h
/**
 * @file MyStructs.h
 * @brief Header file for the structs used in the communication interface
 *  class, responsible for managing motor control and communication.
 */
#ifndef MYSTRUCTS_H
#define MYSTRUCTS_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @defgroup Structs
 * @brief Structs for storing different properties and setpoints of the controllers
 * @{
 */

/**
 * @struct OdoBroadcastStatus
 * @brief Represents the status of odometry data broadcasting.
 */
struct OdoBroadcastStatus
{
    bool angleBroadcast = false; ///< Indicates if angle data is being broadcast.
    bool speedBroadcast = false; ///< Indicates if speed data is being broadcast.
    bool pwmBroadcast = false;   ///< Indicates if PWM data is being broadcast.
};

/**
 * @struct PIDConstants
 * @brief Contains PID constants for motor control.
 */
struct PIDConstants
{
    float p = 0; ///< Proportional constant.
    float i = 0; ///< Integral constant.
    float d = 0; ///< Derivative constant.
};

/**
 * @struct PWMLimits
 * @brief Defines the PWM limits for motor control.
 */
struct PWMLimits
{
    int8_t minPWM = 0; ///< Minimum PWM value.
    int8_t maxPWM = 0; ///< Maximum PWM value.
};

/**
 * @struct MotorConnections
 * @brief Represents the hardware connections for a motor.
 */
struct MotorConnections
{
    uint8_t dirPin;  ///< Direction pin.
    uint8_t pwmPin;  ///< PWM pin.
    uint8_t encPinA; ///< Encoder pin A.
    uint8_t encPinB; ///< Encoder pin B.
};

/**
 * @struct OdometryData
 * @brief Stores odometry information for a motor.
 */
struct OdometryData
{
    float angle = 0; ///< Current angle in degrees.
    float rpm = 0;   ///< Current speed in RPM.
};

/**
 * @struct Setpoint
 * @brief Represents desired values for motor control.
 */
struct Setpoint
{
    float angle = 0; ///< Desired angle in degrees.
    float rpm = 0;   ///< Desired speed in RPM.
};

/**
 * @brief ControlMode enum
 * @brief Enumerates different control modes for motor operation.
 */
enum ControlMode
{
    POSITION_CONTROL,   ///< Position control mode.
    SPEED_CONTROL,      ///< Speed control mode.
    PWM_DIRECT_CONTROL, ///< Direct PWM control mode.
    OFF                 ///< Off

};

/**
 * @struct MotorData
 * @brief Encapsulates all data related to a motor.
 */
struct MotorData
{
    uint8_t motorID = 0;                          ///< Unique identifier for the motor.
    ControlMode controlMode = PWM_DIRECT_CONTROL; ///< Control mode for the motor.
    PIDConstants anglePIDConstants;               ///< PID constants for angle control.
    PIDConstants speedPIDConstants;               ///< PID constants for speed control.
    PWMLimits pwmLimits;                          ///< PWM limits.
    MotorConnections motorConnections;            ///< Hardware connections.
    OdometryData odometryData;                    ///< Odometry data.
    Setpoint setpoint;                            ///< Desired setpoints.
    uint16_t pwmValue = 0;                        ///< Current PWM value.
};

/**
 * @struct ControllerProperties
 * @brief Defines properties of the motor controller.
 */
struct ControllerProperties
{
    bool run = false;                      ///< Indicates if the controller is active.
    uint8_t numMotors = 2;                 ///< Number of motors controlled.
    uint16_t anglePIDFrequency = 30;       ///< Frequency for angle PID updates (Hz).
    uint16_t speedPIDFrequency = 30;       ///< Frequency for speed PID updates (Hz).
    OdoBroadcastStatus odoBroadcastStatus; ///< Status of odometry broadcasting.
};

/**
 * @struct ControllerData
 * @brief Contains all data for the motor controller.
 */
struct ControllerData
{
    MotorData *motorData = nullptr;            ///< Pointer to an array of MotorData structures.
    ControllerProperties controllerProperties; ///< Properties of the controller.
};

struct TaskHandles
{
    TaskHandle_t *wheel_run_task_handles;
};

/** @} */
#endif // MYSTRUCTS_H
