#pragma once
#include "main.h"


// Helper Functions
// Unload matchloader
void unloadMatchloader(Length x, bool useDelay = true, Angle angleOffset = 0_deg);
// Put descore wing in goal from long goal scoring position
void hookDescoreWing(Length x = 0_ft, Length y = 21_in);

// Routes
// Left Auton
void left(bool secureLongGoal = false);
// Left AWP Assist Auton
void leftAwp(bool nineBall = false);
// Right Auton
void right(bool secureLongGoal = false);
// Right AWP Assist Auton
void rightAwp(bool nineBall = false);
// Solo AWP Auton
void soloAwp();
void soloAwp2();

// Skills
void skills();

// For turn pid
void turnTuning(); 

// For translation pid
void translationTuning();

// For finding MAX_SPEED_INCHES_PER_SECOND in speeds.hpp
void maxSpeedMeasuring();

// For testing experimental functions
void testingAuton();
