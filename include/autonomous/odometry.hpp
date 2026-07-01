#pragma once
#include "main.h"
#include "lemlib/api.hpp"


// Default constants
#define TURN_DEFAULT_TIMEOUT 800_ms
#define TRANSLATE_DEFAULT_TIMEOUT 1000_ms
#define TRANSLATE_TIMEOUT_LONG 4000_ms
#define ZERO_SPEED 0_x
#define MAX_SPEED 1_x
#define ZERO_EARLY_EXIT_ANGLE 0_deg
#define ZERO_EARLY_EXIT_RANGE 0_in

// Odom sensors
extern pros::Rotation trackingWheelLRSensor;
// extern pros::Rotation trackingWheelFBSensor;
extern lemlib::TrackingWheel trackingWheelLR; // Left-right motion tracking
// extern lemlib::TrackingWheel trackingWheelFB; // Forward-back motion tracking
extern pros::Imu imu;
extern lemlib::OdomSensors sensors;

// Movement PIDs
extern lemlib::ControllerSettings chassisTurnPid; // Turning based on imu reading
extern lemlib::ControllerSettings chassisDrivePid; // Translations

// True if any drive motor is overheating
bool isOverheating();

// Set chassis position using units library
void setChassisPose(Length x, Length y);
// Set chassis position using units library
void setChassisPose(Length x, Length y, Angle theta);

// Current chassis X position using units library
Length chassisX();
// Current chassis Y position using units library
Length chassisY();
// Current chassis angle using units library
Angle chassisAngle();
// Current chassis pitch using units library
Angle chassisPitch();

// Motor power setting using units library
void setMotorSpeed(pros::Motor motor, Speed speed);
// Chassis power setting using units library
void setChassisSpeed(Speed left, Speed right);

// Delay using units library
void delay(Time time);

// Current time using units library
Time getTime();

extern Time autonStartTime;

// Log position and freeze auton
void printPositionDebug();

// Re-constrain angle to be within (-180_deg, 180_deg)
Angle reconstrainAngle(Angle angle);

// Turn to face any angle
void turnToPoint(Length x, Length y, bool forwards = true, Speed minSpeed = ZERO_SPEED, Speed maxSpeed = MAX_SPEED, Time time = TURN_DEFAULT_TIMEOUT, Angle earlyExitAngle = ZERO_EARLY_EXIT_ANGLE, bool async = false);
// Turn to face any point
void turnToAngle(Angle heading, Speed minSpeed = ZERO_SPEED, Speed maxSpeed = MAX_SPEED, Time time = TURN_DEFAULT_TIMEOUT, Angle earlyExitAngle = ZERO_EARLY_EXIT_ANGLE, bool async = false);
// Swing to face any angle
void swingToPoint(Length x, Length y, DriveSide lockedSide, bool forwards = true, Speed minSpeed = ZERO_SPEED, Speed maxSpeed = MAX_SPEED, Time time = TURN_DEFAULT_TIMEOUT, Angle earlyExitAngle = ZERO_EARLY_EXIT_ANGLE, bool async = false);
// Swing to face any point
void swingToAngle(Angle heading, DriveSide lockedSide, Speed minSpeed = ZERO_SPEED, Speed maxSpeed = MAX_SPEED, Time time = TURN_DEFAULT_TIMEOUT, Angle earlyExitAngle = ZERO_EARLY_EXIT_ANGLE, bool async = false);

// Turn to any angle
void moveToPose(Length x, Length y, Angle heading, bool forwards = true, float carrotMultiplier = 0.6, Speed minSpeed = ZERO_SPEED, Speed maxSpeed = MAX_SPEED, Time time = TRANSLATE_DEFAULT_TIMEOUT, Length earlyExitRange = ZERO_EARLY_EXIT_RANGE, bool async = false);
// Turn to face any point
void moveToPoint(Length x, Length y, bool forwards = true, Speed minSpeed = ZERO_SPEED, Speed maxSpeed = MAX_SPEED, Time time = TRANSLATE_DEFAULT_TIMEOUT, Length earlyExitRange = ZERO_EARLY_EXIT_RANGE, bool async = false);

// Translate forward/backward any amount of distance, regardless of position
void relative(Length distance, Speed minSpeed = ZERO_SPEED, Speed maxSpeed = MAX_SPEED, Time time = TRANSLATE_DEFAULT_TIMEOUT, bool async = false);

// Move on a straight line onto a particular x coordinate, clamping the destination between two y coordinates
void moveToX(Length x, Length yMinimum, Length yMaximum, Speed minSpeed = 0_x, Speed maxSpeed = 1_x, Time time = TRANSLATE_DEFAULT_TIMEOUT, bool async = false);

// Drive at a fixed power for a fixed time, adjusting to maintain a fixed heading
void driveStraightAtAngle(Speed speed, Speed speedChangePerDegreeOffset, Angle angle, Time time);
