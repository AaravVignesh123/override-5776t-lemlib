#include "main.h"
#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"
#include "autonomous/odometry.hpp"
#include "autonomous/odomDistance.hpp"
#include "lemlib/api.hpp"


// ----------------------------------------------------------------------------------------------------
// --- Miscellaneous functions ------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------


void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void pneumaticToggle(int port, bool toggle) {
	if (toggle) {
		pros::c::adi_digital_write(port, HIGH);
	} else {
		pros::c::adi_digital_write(port, LOW);
	}
}

double arrMean(double arr[], int arrLength) {
	double sum = 0;
	double length = arrLength;
	for (int i = 0; i < length; i++) {
        sum += arr[i];
    }
	return sum / length;
}


// ----------------------------------------------------------------------------------------------------
// --- Auton selector ---------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------


enum AutonChars {
	LEFT = 'l',
	LEFT_AWP = 'L',
	RIGHT = 'r',
	RIGHT_AWP = 'R',
	SOLO_AWP = 'W',
	SKILLS = 's',
	TURN_PID = 't',
	DRIVE_PID = 'd',
    MAX_SPEED_AUTON = 'S',
	TESTING_AUTON = 'e',
};

char auton = SOLO_AWP;
bool specialAutonMode = false;

// Initialize auton selector menus
void autonSelector();
void matchAutons();
void leftAuton();
void rightAuton();
void debugAutons();
void noneButton();
void setSpecialAutonMode();
void unsetSpecialAutonMode();

// Auton selector callback functions
void leftSelect() {
	pros::lcd::set_text(2, "Selected Left");
	pros::lcd::set_text(3, "Back / 4 ball / 7 ball");
    pros::lcd::set_text(4, specialAutonMode ? "Mode: <<<<<" : "Mode: >>>>>");
	pros::lcd::register_btn0_cb(leftAuton);
	pros::lcd::register_btn1_cb(setSpecialAutonMode);
	pros::lcd::register_btn2_cb(unsetSpecialAutonMode);
	auton = LEFT;
}
void leftAwpSelect() {
	pros::lcd::set_text(2, "Selected Left AWP");
	pros::lcd::set_text(3, "Back / 9 ball / 7 ball");
    pros::lcd::set_text(4, specialAutonMode ? "Mode: <<<<<" : "Mode: >>>>>");
	pros::lcd::register_btn0_cb(leftAuton);
	pros::lcd::register_btn1_cb(setSpecialAutonMode);
	pros::lcd::register_btn2_cb(unsetSpecialAutonMode);
	auton = LEFT_AWP;
}
void rightSelect() {
	pros::lcd::set_text(2, "Selected Right");
	pros::lcd::set_text(3, "Back / 4 ball / 7 ball");
    pros::lcd::set_text(4, specialAutonMode ? "Mode: <<<<<" : "Mode: >>>>>");
	pros::lcd::register_btn0_cb(rightAuton);
	pros::lcd::register_btn1_cb(setSpecialAutonMode);
	pros::lcd::register_btn2_cb(unsetSpecialAutonMode);
	auton = RIGHT;
}
void rightAwpSelect() {
	pros::lcd::set_text(2, "Selected Right AWP");
	pros::lcd::set_text(3, "Back / 9 ball / 7 ball");
	pros::lcd::register_btn0_cb(rightAuton);
	pros::lcd::register_btn1_cb(noneButton);
	pros::lcd::register_btn2_cb(noneButton);
	auton = RIGHT_AWP;
}
void soloAwpSelect() {
	pros::lcd::set_text(2, "Selected Solo AWP");
	pros::lcd::set_text(3, "Back /    /");
	pros::lcd::register_btn0_cb(matchAutons);
	pros::lcd::register_btn1_cb(noneButton);
	pros::lcd::register_btn2_cb(noneButton);
	auton = SOLO_AWP;
}
void skillsSelect() {
	pros::lcd::set_text(2, "Selected Skills");
	pros::lcd::set_text(3, "Back /    /");
	pros::lcd::register_btn0_cb(autonSelector);
	pros::lcd::register_btn1_cb(noneButton);
	pros::lcd::register_btn2_cb(noneButton);
	auton = SKILLS;
}
void turnTuningSelect() {
	pros::lcd::set_text(2, "Selected Turn Tuning");
	pros::lcd::set_text(3, "Back /    /");
	pros::lcd::register_btn0_cb(debugAutons);
	pros::lcd::register_btn1_cb(noneButton);
	pros::lcd::register_btn2_cb(noneButton);
	auton = TURN_PID;
}
void driveTuningSelect() {
	pros::lcd::set_text(2, "Selected Drive Tuning");
	pros::lcd::set_text(3, "Back /    /");
	pros::lcd::register_btn0_cb(debugAutons);
	pros::lcd::register_btn1_cb(noneButton);
	pros::lcd::register_btn2_cb(noneButton);
	auton = DRIVE_PID;
}
void testingSelect() {
	pros::lcd::set_text(2, "Selected Testing Auton");
	pros::lcd::set_text(3, "Back /    /");
	pros::lcd::register_btn0_cb(debugAutons);
	pros::lcd::register_btn1_cb(noneButton);
	pros::lcd::register_btn2_cb(noneButton);
	auton = TESTING_AUTON;
}

// Auton selector menus
void autonSelector() {
	pros::lcd::set_text(1, "Auton mode selector panel:");
	pros::lcd::set_text(2, "Match / Skills / Debug");
	pros::lcd::clear_line(3);
	pros::lcd::register_btn0_cb(matchAutons);
	pros::lcd::register_btn1_cb(skillsSelect);
	pros::lcd::register_btn2_cb(debugAutons);
}

void withAllianceAutons() {
	pros::lcd::set_text(1, "Single side auton selector panel:");
	pros::lcd::set_text(2, "Left / Right / Back");
	pros::lcd::clear_line(3);
	pros::lcd::register_btn0_cb(leftAuton);
	pros::lcd::register_btn1_cb(rightAuton);
	pros::lcd::register_btn2_cb(matchAutons);
}

void matchAutons() {
	pros::lcd::set_text(1, "Match auton selector panel:");
	pros::lcd::set_text(2, "Other / Solo AWP / Back");
	pros::lcd::clear_line(3);
	pros::lcd::register_btn0_cb(withAllianceAutons);
	pros::lcd::register_btn1_cb(soloAwpSelect);
	pros::lcd::register_btn2_cb(autonSelector);
}

void leftAuton() {
	pros::lcd::set_text(1, "Left auton selector panel:");
	pros::lcd::set_text(2, "Normal / AWP / Back");
	pros::lcd::clear_line(3);
    pros::lcd::clear_line(4);
	pros::lcd::register_btn0_cb(leftSelect);
	pros::lcd::register_btn1_cb(leftAwpSelect);
	pros::lcd::register_btn2_cb(withAllianceAutons);
}

void rightAuton() {
	pros::lcd::set_text(1, "Right auton selector panel:");
	pros::lcd::set_text(2, "Normal / AWP / Back");
	pros::lcd::clear_line(3);
    pros::lcd::clear_line(4);
	pros::lcd::register_btn0_cb(rightSelect);
	pros::lcd::register_btn1_cb(rightAwpSelect);
	pros::lcd::register_btn2_cb(withAllianceAutons);
}

void debugAutons() {
	pros::lcd::set_text(1, "Debug auton selector panel:");
	pros::lcd::set_text(2, "Turn PID / Drive PID / Testing");
	pros::lcd::clear_line(3);
	pros::lcd::register_btn0_cb(turnTuningSelect);
	pros::lcd::register_btn1_cb(driveTuningSelect);
	pros::lcd::register_btn2_cb(testingSelect);
}

void noneButton() {}

void setSpecialAutonMode() {
    specialAutonMode = true;
    pros::lcd::set_text(4, specialAutonMode ? "Mode: <<<<<" : "Mode: >>>>>");
}
void unsetSpecialAutonMode() {
    specialAutonMode = false;
    pros::lcd::set_text(4, specialAutonMode ? "Mode: <<<<<" : "Mode: >>>>>");
}


// ----------------------------------------------------------------------------------------------------
// --- Driver Code Variables --------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------


struct controllerButton {
	bool pressed = false;
	bool held = false;
	bool released = false;
    bool wasHeld = false;
};

// Pneumatic activation controls
bool scraperToggle;
bool descoreToggle;
bool middleDescoreToggle;

// Controller inputs
pros::Controller controller(pros::E_CONTROLLER_MASTER);
int leftX;
int leftY;
int rightX;
int rightY;
controllerButton xButton;
controllerButton yButton;
controllerButton aButton;
controllerButton bButton;
controllerButton upButton;
controllerButton downButton;
controllerButton leftButton;
controllerButton rightButton;
controllerButton l1Button;
controllerButton l2Button;
controllerButton r1Button;
controllerButton r2Button;

void pollControllerDigital(controllerButton& button, pros::controller_digital_e_t buttonCode) {
    button.held = controller.get_digital(buttonCode);
    button.pressed = button.held && button.held != button.wasHeld;
    button.released = !button.held && button.held != button.wasHeld;
    button.wasHeld = button.held;
}

// Debug
bool holdDrive = false;
int showDebug = 0;

// Macro variables
bool enableMacros = false;
Time currentTime;
enum intakeStates {
    STOPPED = 'S',
    STORE_BALLS = 'H',
    LONG_GOAL = 'L',
    MID_GOAL = 'M',
    LOW_GOAL = 'B'
};
intakeStates intakeCommand = STOPPED;
intakeStates lastIntakeCommand = STOPPED;
double tipAngle = 0.0;
double lastTipAngles[10] = {};  // Mean filter of last 10 angles for noise reduction
int lastTipAnglesSize = sizeof(lastTipAngles) / sizeof(lastTipAngles[0]);
double tipMagnitude = 0.0;
double tipSign = 1.0;
double antiTipCommand = 0.0;
double tipVelocity = 0.0;

// Position recording
lemlib::Pose pose = chassis.getPose();
double poseX;
double poseY;


// ----------------------------------------------------------------------------------------------------
// --- Competition functions --------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------


void initialize() {
    // Reset tracking wheel position
	trackingWheelLR.reset();
	// trackingWheelFB.reset();

	pros::lcd::initialize();
    chassis.calibrate();

	// Initialize pneumatics
	pros::c::adi_pin_mode(kPneumaticScraperPort, OUTPUT);
	pros::c::adi_pin_mode(kPneumaticDescorePort, OUTPUT);
	pros::c::adi_pin_mode(kPneumaticGatePort, OUTPUT);
    // pros::c::adi_pin_mode(kPneumaticIntakePort, OUTPUT);
    pros::c::adi_pin_mode(kPneumaticMiddleDescorePort, OUTPUT);

	autonSelector();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	allMotors.set_brake_mode_all(pros::MotorBrake::hold); // for tighter movements
    intakeTop.set_brake_mode(pros::MotorBrake::hold);

    autonStartTime = getTime();

	switch (auton) {
		case LEFT:
			left(specialAutonMode);
			break;
		case LEFT_AWP:
			leftAwp(specialAutonMode);
			break;
		case RIGHT:
			right(specialAutonMode);
			break;
		case RIGHT_AWP:
			rightAwp(specialAutonMode);
			break;
		case SOLO_AWP:
			soloAwp2();
			break;
		case SKILLS:
		    skills(); //skillsBasic();
			break;
		case TURN_PID:
			turnTuning();
			break;
		case DRIVE_PID:
			translationTuning();
			break;
        case MAX_SPEED_AUTON:
            maxSpeedMeasuring();
            break;
		case TESTING_AUTON:
			testingAuton();
			break;
	}
}


void opcontrol() {
	Time init = getTime() / 1000;

    // Set control variables
    scraperToggle = false; //pros::c::adi_digital_read(kPneumaticScraperPort);
    descoreToggle = false;
    middleDescoreToggle = false;
	
	// Setting all motors to coast
	allMotors.set_brake_mode_all(pros::MotorBrake::coast);

    // Floor info
    controller.set_text(0, 1, "Floor Up  ");

    chassis.cancelAllMotions();

    // Debug menu
    pros::Task screenTask([&]() {
        while (true) {
            // if (pros::Task::notify_take(true, TIMEOUT_MAX)) break;

            switch (showDebug) {
                case 1: {
                    pros::lcd::set_text(0, "LF: " + std::to_string(driveLF.get_position()));
                    pros::lcd::set_text(1, "LC: " + std::to_string(driveLC.get_position()));
                    pros::lcd::set_text(2, "LB: " + std::to_string(driveLB.get_position()));
                    pros::lcd::set_text(3, "RF: " + std::to_string(driveRF.get_position()));
                    pros::lcd::set_text(4, "RC: " + std::to_string(driveRC.get_position()));
                    pros::lcd::set_text(5, "RB: " + std::to_string(driveRB.get_position()));
                    pros::lcd::set_text(6, "Position (ft): (" + std::to_string(chassisX().convert(foot)) + "," + std::to_string(chassisY().convert(foot)) + ")");
                    pros::lcd::set_text(7, "Angle: " + std::to_string(chassisAngle().convert(degree)) + " deg");
                    break;
                } case 2: {
                    pros::lcd::set_text(0, "Up:    " + std::to_string(upButton.held));
                    pros::lcd::set_text(1, "Down:  " + std::to_string(downButton.held));
                    pros::lcd::set_text(2, "Left:  " + std::to_string(leftButton.held));
                    pros::lcd::set_text(3, "Right: " + std::to_string(rightButton.held));
                    pros::lcd::set_text(4, "X: " + std::to_string(xButton.held));
                    pros::lcd::set_text(5, "Y: " + std::to_string(yButton.held));
                    pros::lcd::set_text(6, "A: " + std::to_string(aButton.held));
                    pros::lcd::set_text(7, "B: " + std::to_string(bButton.held));
                    break;
                } case 3: {
                    pros::lcd::set_text(0, "L1: " + std::to_string(l1Button.held));
                    pros::lcd::set_text(1, "L2: " + std::to_string(l2Button.held));
                    pros::lcd::set_text(2, "R1: " + std::to_string(r1Button.held));
                    pros::lcd::set_text(3, "R2: " + std::to_string(r2Button.held));
                    pros::lcd::set_text(4, "Left stick:  (" + std::to_string(leftX).substr(0, 4) + ", " + std::to_string(leftY).substr(0, 4) + ")");
                    pros::lcd::set_text(5, "Right stick: (" + std::to_string(rightX).substr(0, 4) + ", " + std::to_string(rightY).substr(0, 4) + ")");
                    // pros::lcd::set_text(6, "FB Velocity: " + std::to_string(trackingWheelFBSensor.get_velocity()));
                    pros::lcd::clear_line(6);
                    pros::lcd::clear_line(7);
                    break;
                } case 4: {
                    pros::lcd::set_text(0, "Tipping angle:  " + std::to_string(tipAngle));
                    pros::lcd::set_text(1, "Scraper down:   " + std::to_string(scraperToggle));
                    pros::lcd::set_text(2, "Intake command: " + std::to_string(intakeCommand));
                    pros::lcd::set_text(3, "Right descore:  " + std::to_string(descoreToggle));
                    pros::lcd::set_text(4, "Distance:       " + std::to_string(odomDistanceForward.getDistance().convert(inch)));
                    pros::lcd::set_text(5, "Is Overheating: " + std::to_string(isOverheating()));
                    pros::lcd::set_text(6, "Enable macros:  " + std::to_string(enableMacros));
                    pros::lcd::set_text(7, "Time:           " + std::to_string(currentTime.convert(second)));
                    break;
                } case 5: {
                    pros::lcd::set_text(0, "LF Power Draw: " + std::to_string(driveLF.get_current_draw()));
                    pros::lcd::set_text(1, "LC Power Draw: " + std::to_string(driveLC.get_current_draw()));
                    pros::lcd::set_text(2, "LB Power Draw: " + std::to_string(driveLB.get_current_draw()));
                    pros::lcd::set_text(3, "RF Power Draw: " + std::to_string(driveRF.get_current_draw()));
                    pros::lcd::set_text(4, "RC Power Draw: " + std::to_string(driveRC.get_current_draw()));
                    pros::lcd::set_text(5, "RB Power Draw: " + std::to_string(driveRB.get_current_draw()));
                    pros::lcd::set_text(6, "Tipping angle: " + std::to_string(chassisPitch().convert(degree)));
                    pros::lcd::set_text(7, "Untip command: " + std::to_string(antiTipCommand));
                    break;
                } case 0: {
                    for (int i = 0; i <= 7; i++) {
                        pros::lcd::clear_line(i);
                    }
                    pros::lcd::set_text(1, "g"); // g
                    break;
                }
            }

            // If you go much faster than this, the brain throws an error because it's redrawing too quickly.
            delay(100_ms);
        }
    });

    // Intake task
    pros::Task intakeTask([&] {
		while (true) {
            // if (pros::Task::notify_take(true, TIMEOUT_MAX)) break;

            // Gate controls
            pneumaticToggle(kPneumaticGatePort, l1Button.held);

            // Convert intake command into intake motor powers
            switch (intakeCommand) {
                case STOPPED:
                    stopIntake();
                    break;
                case STORE_BALLS:
                    storeBalls();  // Duplicate intake commands to move the gate at the start to avoid shenanigans
                    if (lastIntakeCommand == MID_GOAL) {
                        clearMiddleSection();
                        delay(CLEAR_MIDDLE_SECTION);
                        storeBalls();
                    }
                    break;
                case LONG_GOAL:
                    scoreBallsUp();
                    if (lastIntakeCommand == MID_GOAL) {
                        clearMiddleSection();
                        delay(CLEAR_MIDDLE_SECTION);
                        scoreBallsUp();
                    }

                    // Score 2 macro
                    if (enableMacros) {
                        delay(TWO_BALLS_INTO_LONG);
                        pros::c::adi_digital_write(kPneumaticGatePort, LOW);
                        scoreBallsDown();
                        delay(200_ms);
                        storeBalls();
                        delay(100_ms);
                    }
                    break;
                case MID_GOAL:
                    scoreBallsMiddle(true);
                    if (lastIntakeCommand != MID_GOAL) {
                        scoreBallsDown();
                        delay(CLEAR_HOOD_SECTION);
                        scoreBallsMiddle(true);
                    }
                    break;
                case LOW_GOAL:
                    if (lastIntakeCommand == MID_GOAL) {
                        clearMiddleSection();
                        delay(CLEAR_MIDDLE_SECTION);
                    }
                    scoreBallsDown();
                    break;
            }

            // Don't update lastIntakeCommand if the last command was a stopped intake (the balls haven't moved)
            // lastIntakeCommand is used to know if we should clear the middle section of a ball or not when scoring elsewhere
            // or whether we should clear the hood area of a ball or not when scoring into the middle goal
            if (intakeCommand != STOPPED) {
                lastIntakeCommand = intakeCommand;
            }

            delay(10_ms);
		}
  	});

	// Main loop
	while (true) {
		currentTime = getTime() - init;
		pose = chassis.getPose();

		// Poll controller analog
		leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// Poll controller digital
        pollControllerDigital(xButton, pros::E_CONTROLLER_DIGITAL_X);
        pollControllerDigital(yButton, pros::E_CONTROLLER_DIGITAL_Y);
        pollControllerDigital(aButton, pros::E_CONTROLLER_DIGITAL_A);
        pollControllerDigital(bButton, pros::E_CONTROLLER_DIGITAL_B);
        pollControllerDigital(upButton, pros::E_CONTROLLER_DIGITAL_UP);
        pollControllerDigital(downButton, pros::E_CONTROLLER_DIGITAL_DOWN);
        pollControllerDigital(leftButton, pros::E_CONTROLLER_DIGITAL_LEFT);
        pollControllerDigital(rightButton, pros::E_CONTROLLER_DIGITAL_RIGHT);
        pollControllerDigital(l1Button, pros::E_CONTROLLER_DIGITAL_L1);
        pollControllerDigital(l2Button, pros::E_CONTROLLER_DIGITAL_L2);
        pollControllerDigital(r1Button, pros::E_CONTROLLER_DIGITAL_R1);
        pollControllerDigital(r2Button, pros::E_CONTROLLER_DIGITAL_R2);

		// Macros
		enableMacros = aButton.held;

		// Intake controls
		if (r2Button.held && l1Button.held) {
            intakeCommand = LONG_GOAL;
		} else if (r2Button.held && l2Button.held) {
            intakeCommand = MID_GOAL;
		} else if (r1Button.held) {
            intakeCommand = LOW_GOAL;
		} else if (r2Button.held) {
			intakeCommand = STORE_BALLS;
		} else {
			intakeCommand = STOPPED;
		}

		// Scraper toggle
		if (!enableMacros && bButton.pressed) {
			scraperToggle = !scraperToggle;
			pneumaticToggle(kPneumaticScraperPort, scraperToggle);
		}

		// Descore toggle
        descoreToggle = rightButton.held;
        pneumaticToggle(kPneumaticDescorePort, descoreToggle);

        // Middle descore toggle
        middleDescoreToggle = l2Button.held;
        pneumaticToggle(kPneumaticMiddleDescorePort, middleDescoreToggle);

		// Debug things
		if (xButton.pressed) {
			showDebug = (showDebug + 1) % 6;
		}

        // if (auton == SKILLS) {
        //     pneumaticToggle(kPneumaticDescorePort, true);
        // }

        if (enableMacros && auton != SKILLS && leftButton.held) {
            hookDescoreWing();

            // Effectively have a 1s timeout on the final wing motion
            delay(1_s);
            chassis.cancelAllMotions();
		}
		
		// Anti tip protection macro
        // Anti tip is currently disabled
		tipAngle = 0; //imuTipping.controllerGet();
		tipMagnitude = abs(tipAngle);
		tipSign = std::signbit(tipAngle) ? 1 : -1;
		tipVelocity = (tipAngle - arrMean(lastTipAngles, lastTipAnglesSize)) * tipSign;  // Positive if tipping, negative if untipping

		// Lock out anti tip after 75 seconds (30 seconds remaining)
		if (tipMagnitude > 10 && currentTime < 75_s) {
			// Calculate anti tip command
			// Untip harder if we're tipping faster (velocity * 0.1)
			// Untip harder if we're tipped further from zero (angle * 0.05)
			antiTipCommand = std::max(0.0, tipVelocity) * 0.03 + tipMagnitude * 0.05;
			antiTipCommand *= tipSign;

			// Send anti tip command to drive controls
			leftY = antiTipCommand;
			rightY = antiTipCommand;
		} else {
			antiTipCommand = 0;
		}

		// Shift all anti-tip angle values over by one
		for (int i = lastTipAnglesSize - 1; i > 0; i--) {
			lastTipAngles[i] = lastTipAngles[i - 1];
		}
		lastTipAngles[0] = tipAngle;

		// Tank controls
        chassis.tank(leftY, rightY);
        // 2-stick arcade controls
        // chassis.arcade(leftY, rightX);

		delay(10_ms);
	}
}
