#pragma once
#include "main.h"
#include "lemlib/api.hpp"


// Drive motor ports
extern const int8_t kDriveLFPort;
extern const int8_t kDriveLCPort;
extern const int8_t kDriveLBPort;
extern const int8_t kDriveRFPort;
extern const int8_t kDriveRCPort;
extern const int8_t kDriveRBPort;

extern pros::Motor driveLF;
extern pros::Motor driveLC;
extern pros::Motor driveLB;
extern pros::Motor driveRF;
extern pros::Motor driveRC;
extern pros::Motor driveRB;
extern pros::MotorGroup allMotors;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// Drivetrain
extern lemlib::Drivetrain drivetrain;

// Chassis
extern lemlib::Chassis chassis;
