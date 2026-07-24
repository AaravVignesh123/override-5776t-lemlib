#include "main.h"
#include "subsystems/intake.hpp"
#include "autonomous/odometry.hpp"
#include "subsystems/pneumatics.hpp"

pros::Motor intakeBottomLarge(21); // Lower 11W intake motor with port
pros::Motor intakeBottomSmall(-5); // Lower 5.5W intake motor with port
pros::Motor intakeTop(21); // Top intake motor with port

void storeBalls() {
    setMotorSpeed(intakeBottomLarge, 1_x);
    setMotorSpeed(intakeBottomSmall, 1_x);
    setMotorSpeed(intakeTop, 0.4_x);
    pros::c::adi_digital_write(kPneumaticGatePort, LOW);
}

void scoreBallsUp() {
    setMotorSpeed(intakeBottomLarge, 1_x);
    setMotorSpeed(intakeBottomSmall, 1_x);
    setMotorSpeed(intakeTop, 1_x);
    pros::c::adi_digital_write(kPneumaticGatePort, HIGH);
}

void scoreBallsMiddle(bool driverControl) {
    setMotorSpeed(intakeBottomLarge, 1_x);
    setMotorSpeed(intakeBottomSmall, 1_x);
    setMotorSpeed(intakeTop, driverControl ? -0.4_x : -0.3_x);
    pros::c::adi_digital_write(kPneumaticMiddleDescorePort, HIGH);
    pros::c::adi_digital_write(kPneumaticGatePort, LOW);
}

void clearMiddleSection() {
    setMotorSpeed(intakeBottomLarge, -1_x);
    setMotorSpeed(intakeBottomSmall, -1_x);
    setMotorSpeed(intakeTop, 0.5_x);
}

void scoreBallsDown() {
    setMotorSpeed(intakeBottomLarge, -1_x);
    setMotorSpeed(intakeBottomSmall, -1_x);
    setMotorSpeed(intakeTop, -1_x);
}

void stopIntake() {
    setMotorSpeed(intakeBottomLarge, 0_x);
    setMotorSpeed(intakeBottomSmall, 0_x);
    setMotorSpeed(intakeTop, 0_x);
}
