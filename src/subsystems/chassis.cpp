#include "subsystems/chassis.hpp"
#include "autonomous/odometry.hpp"
#include "lemlib/api.hpp"

// Motor ports, motors, and motor groups

const int8_t kDriveLFPort = -13;
const int8_t kDriveLCPort = -12;
const int8_t kDriveLBPort = 11;
const int8_t kDriveRFPort = 18;
const int8_t kDriveRCPort = 19;
const int8_t kDriveRBPort = -14;
pros::Motor driveLF(kDriveLFPort);
pros::Motor driveLC(kDriveLCPort);
pros::Motor driveLB(kDriveLBPort);
pros::Motor driveRF(kDriveRFPort);
pros::Motor driveRC(kDriveRCPort);
pros::Motor driveRB(kDriveRBPort);
pros::MotorGroup leftMotors({kDriveLFPort, kDriveLCPort, kDriveLBPort}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({kDriveRFPort, kDriveRCPort, kDriveRBPort}, pros::MotorGearset::blue);
pros::MotorGroup allMotors({kDriveLFPort, kDriveLCPort, kDriveLBPort, kDriveRFPort, kDriveRCPort, kDriveRBPort}, pros::MotorGearset::blue);

// Drivetrain object
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    11.5,  // wheel track width (distance between the centers of the wheels on opposite sides of the drive)
    lemlib::Omniwheel::NEW_325,  // wheel diameter
    450, // drivetrain rpm
    2 // horizontal drift - 2 for full omnis, 8 for traction
);

// Chassis object
lemlib::Chassis chassis(
    drivetrain,
    chassisDrivePid,
    chassisTurnPid,
    sensors
);
