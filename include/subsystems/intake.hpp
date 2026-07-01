#pragma once
#include "main.h"


// Long goal timing constants
#define TWO_BALLS_INTO_LONG 300_ms
#define FOUR_BALLS_INTO_LONG 550_ms

// Matchload timing constants
#define MATCHLOAD_THREE_BLOCKS 600_ms

// Clearing sections of balls constants
#define CLEAR_MIDDLE_SECTION 200_ms
#define CLEAR_HOOD_SECTION 150_ms

// Intake motors
extern pros::Motor intakeBottomLarge;
extern pros::Motor intakeBottomSmall;
extern pros::Motor intakeTop;

void storeBalls();
void scoreBallsUp();
void scoreBallsMiddle(bool driverControl = false);
void clearMiddleSection();
void scoreBallsDown();
void stopIntake();
