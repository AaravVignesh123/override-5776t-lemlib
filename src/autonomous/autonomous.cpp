#include "autonomous/odometry.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/pneumatics.hpp"
#include "autonomous/autonomous.hpp"
#include "autonomous/odomDistance.hpp"

// Helper Functions ---------------------------------------
void unloadMatchloader(Length x, bool useDelay, Angle angleOffset) {
    turnToAngle(180_deg + angleOffset, 0.1_x);
    pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
    if (useDelay) {
        delay(150_ms);
    }
    storeBalls();
    moveToPoint(x, -15_in, true, 0.4_x, 0.7_x);
    setChassisSpeed(0.3_x, 0.3_x);
}

void hookDescoreWing(Length x, Length y) {
    // Initialize long goal position
    setChassisPose(x, y, 180_deg);

    // Hook in descore wing
    stopIntake();
    turnToAngle(115_deg, 1_x);
    swingToAngle(150_deg, lemlib::DriveSide::RIGHT, 0.7_x);
    // delay(150_ms);
    moveToPoint(x + 9_in, y + 23_in, false, 0.6_x, 1_x, 0.4_s);
    moveToPoint(x + 9_in, y + 23_in, false, 0_x, 0.6_x, 15_s, 0_in, true);  // No timeout to continue reversing until auton time ends
}

// Macro to align into the long goal using driveStraightAtAngle(-0.5_x, 0.08_x, 180_deg, [time]);
#define ALIGN_TO_LONG_GOAL(time) driveStraightAtAngle(-0.6_x, 0.1_x, 180_deg, time);
#define ALIGN_TO_MIDDLE_GOAL(time) driveStraightAtAngle(-0.15_x, 0.02_x, -135_deg, time);

// Left Side Autonomous Route -----------------------------
void left(bool secureLongGoal) {
    // Initialize starting position
    setChassisPose(-3.5_in, 0.5_in, 0_deg);
    odomDistanceForward.setOriginPosition(-1_ft, -4_ft);
    odomDistanceRight.setOriginPosition(-1_ft, -4_ft);

    if (secureLongGoal) {
        // 4 balls in the long goal rush

        // Grab 3 balls by top middle goal
        storeBalls();
        moveToPose(-1_ft + 0.5_in, 2_ft - 0.5_in, -5_deg, true, 0.5, 0.6_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        
        // Score 4 balls into left long goal
        turnToAngle(-115_deg, 0.4_x, 1_x, 2_s);
        pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
        setChassisSpeed(1_x, 1_x);
        delay(250_ms);
        moveToX(-3_ft + 1_in, -3_in, 9_in, 0.4_x, 1_x, 2_s);
        turnToAngle(-165_deg, 0.4_x);
        // odomDistanceRight.correctPosition();
        moveToPoint(-3_ft - 1_in, 14_in, false, 0.8_x);
        scoreBallsUp();
        ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)

        // Hook descore arm into goal to secure control bonus
        hookDescoreWing(-3_ft, 20_in);

        // Hyperspeed 4 ball (lowest priority)
        /*
        // Grab 3 balls by top middle goal
        storeBalls();
        moveToPose(-1_ft + 0.5_in, 2_ft - 1_in, -15_deg, true, 0.5, 0.6_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        
        // Score 4 balls into left long goal
        turnToAngle(70_deg, 0.4_x);
        moveToX(-3_ft + 4_in, 6_in, 1_ft + 3_in, 0.5_x);
        swingToAngle(175_deg, lemlib::DriveSide::LEFT, 0.5_x);
        moveToPoint(-3_ft + 1_in, 11_in, false, 0.8_x);
        scoreBallsUp();
        ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG + 350_ms)
        pros::c::adi_digital_write(kPneumaticScraperPort, LOW);

        // Hook descore arm into goal to secure control bonus
        hookDescoreWing(-3_ft, 20_in);
        */
    } else {
        // 7 balls in the long goal rush

        // Grab 3 balls by top middle goal
        storeBalls();
        moveToPose(-1_ft + 0.5_in, 2_ft - 3_in, -13_deg, true, 0.5, 0.6_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        
        // Unload 3 balls from left matchloader
        turnToAngle(-115_deg, 0.4_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
        setChassisSpeed(1_x, 1_x);
        delay(250_ms);
        moveToX(-3_ft + 8_in, 4_in, 15_in, 0.6_x);
        // printPositionDebug();
        // turnToAngle(-175_deg, 0.4_x);
        // odomDistanceRight.correctPosition();
        moveToPoint(-3_ft + 2_in, -12_in, true, 0.4_x, 0.7_x);
        setChassisSpeed(0.3_x, 0.3_x);
        delay(MATCHLOAD_THREE_BLOCKS);
        odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);

        // Score 7 balls in left long goal
        moveToPoint(-3_ft + 2_in, 1_ft + 3_in, false, 0.8_x);
        scoreBallsUp();
        ALIGN_TO_LONG_GOAL(1_s)

        // Hook descore arm into goal to secure control bonus
        hookDescoreWing(-3_ft, 20_in);
    }
}

// Left Side AWP Assist Autonomous Route ------------------
void leftAwp(bool nineBall) {
    if (nineBall) {
        // 9 balls split 4/5 Long/Middle
        // Initialize starting position
        setChassisPose(-3.5_in, 0.5_in, 0_deg);
        odomDistanceForward.setOriginPosition(-1_ft, -4_ft);
        odomDistanceRight.setOriginPosition(-1_ft, -4_ft);

        // Grab 3 balls by top middle goal
        storeBalls();
        moveToPoint(-1_ft - 2_in, 2_ft, true, 0.5_x, 0.8_x, 1.5_s);

        // Grab 2 blocks from under left long goal
        moveToPose(-3_ft + 1_in, 3_ft + 4_in, -55_deg, true, 0.2, 0.4_x, 0.8_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        delay(100_ms);
        
        // Score 4 balls into left long goal
        setChassisSpeed(0.4_x, -1_x);
        delay(400_ms);
        swingToAngle(0_deg, lemlib::DriveSide::LEFT, 0.8_x);
        moveToPoint(-2_ft - 9_in, 2_ft + 1_in, false, 0.7_x);
        swingToAngle(160_deg, lemlib::DriveSide::LEFT, 1_x);
        swingToAngle(175_deg, lemlib::DriveSide::LEFT, 1_x);
        ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)
        scoreBallsUp();
        ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)

        // Correct position and get into left matchloader
        setChassisPose(-3_ft, 2_ft);
        moveToPose(-3_ft, -13_in, 180_deg, true, 0.4, 0.5_x, 1_x, 1.5_s, 0_in, true);
        scoreBallsDown();
        delay(200_ms);
        storeBalls();
        chassis.waitUntilDone();
    } else {
        // 7 balls split 4/3 Long/Middle
        // Run 4 ball left
        left(true);

        // Wait until as late as possible to leave the long goal
        while (getTime() - autonStartTime < 10_s) {
            delay(20_ms);
        }

        // Exit all motions if we're still trying to wing
        chassis.cancelMotion();

        // Correct position and get into left matchloader
        odomDistanceForward.ignoreTooLargeCorrections = true;
        odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
        odomDistanceForward.ignoreTooLargeCorrections = false;
        odomDistanceRight.correctPosition(OdomDistance::ForceToWall::LEFT_RIGHT);
        // pros::c::adi_digital_write(kPneumaticDescorePort, HIGH);
        moveToPoint(-2_ft - 1_in, 2_ft - 3_in, true, 0.5_x);
        turnToAngle(-165_deg, 0.4_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        storeBalls();
        moveToPose(-3_ft + 2_in, -14.5_in, 180_deg, true, 0.27, 0.5_x, 1_x, 1.5_s);
    }
    
    // Unload 3 blocks from left matchloader
    setChassisSpeed(0.4_x, 0.4_x);
    delay(MATCHLOAD_THREE_BLOCKS);
    odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
    odomDistanceRight.correctPosition(OdomDistance::ForceToWall::LEFT_RIGHT);

    // Get into top middle goal
    moveToPoint(2_in, 3_ft + 2_in, false, 0_x, 0.75_x, 0.5_s);
    // moveToPose(1.5_in, 3_ft - 0.5_in, -135_deg, false, 0.5, 1_x, 1_x, 0.5_s);
    pros::c::adi_digital_write(kPneumaticMiddleDescorePort, HIGH);
    moveToPose(2_in, 3_ft + 2_in, -135_deg, false, 0.6, 0.6_x, 1_x, 0.8_s);
    moveToPoint(2_in, 3_ft + 2_in, false, 0.4_x, 1_x, 1_s, 7.5_in);

    // Score 3 blocks in top middle goal
    // turnToAngle(-135_deg, 0.4_x);
    // setChassisSpeed(-0.25_x, -0.25_x);
    scoreBallsDown();
    delay(CLEAR_HOOD_SECTION);
    scoreBallsMiddle();
    // setChassisSpeed(0_x, 0_x);
    delay(0.5_s);
    // ALIGN_TO_MIDDLE_GOAL(0.5_s)
    pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
    pros::c::adi_digital_write(kPneumaticDescorePort, LOW);
    delay(10_s);
    // ALIGN_TO_MIDDLE_GOAL(5_s)

    // moveToX(-2_ft + 7.8_in, 9_in, 15_in, 0.1_x, 1_x, 2_s);
    // // moveToPoint(-2_ft + 7.5_in, 12_in);
    // turnToAngle(170_deg, 0.4_x);
    // moveToPoint(-2_ft + 1_in, 3_ft + 2_in, false);
    // turnToAngle(180_deg);
    // delay(200_ms);
}

// Right Side Autonomous Route ----------------------------
void right(bool secureLongGoal) {
    // Initialize starting position
    setChassisPose(3.5_in, 0.5_in, 0_deg);
    odomDistanceForward.setOriginPosition(1_ft, -4_ft);
    odomDistanceLeft.setOriginPosition(1_ft, -4_ft);

    if (secureLongGoal) {
        // 4 balls in the long goal rush

        // Grab 3 balls by bottom middle goal
        storeBalls();
        moveToPose(1_ft - 2.5_in, 2_ft - 2_in, 10_deg, true, 0.5, 0.6_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        
        // Score 4 balls into left long goal
        turnToAngle(105_deg, 0.4_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
        setChassisSpeed(1_x, 1_x);
        delay(250_ms);
        moveToX(3_ft - 3_in, -3_in, 9_in, 0.4_x, 1_x, 1.5_s);
        turnToAngle(175_deg, 0.4_x);
        // odomDistanceLeft.correctPosition();
        moveToPoint(3_ft + 1_in, 15_in, false, 0.8_x);
        scoreBallsUp();
        ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)

        // Hook descore arm into goal to secure control bonus
        hookDescoreWing(3_ft, 20_in);

        // Hyperspeed 4 ball (lowest priority)
        /*
        // Grab 3 balls by bottom middle goal
        storeBalls();
        moveToPose(1_ft - 0.5_in, 2_ft - 1_in, 15_deg, true, 0.5, 0.6_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        
        // Score 4 balls into right long goal
        turnToAngle(-70_deg, 0.4_x);
        moveToX(3_ft - 4_in, 6_in, 1_ft + 3_in, 0.5_x);
        swingToAngle(-175_deg, lemlib::DriveSide::LEFT, 0.5_x);
        moveToPoint(3_ft + 1_in, 11_in, false, 0.8_x);
        scoreBallsUp();
        ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG + 350_ms)
        pros::c::adi_digital_write(kPneumaticScraperPort, LOW);

        // Hook descore arm into goal to secure control bonus
        hookDescoreWing(3_ft, 20_in);
        */
    } else {
        // 7 balls in the long goal rush

        // Grab 3 balls by bottom middle goal
        storeBalls();
        moveToPose(1_ft - 0.5_in, 2_ft - 1_in, 15_deg, true, 0.5, 0.6_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        delay(75_ms);
        
        // Unload 3 balls from right matchloader
        turnToAngle(125_deg, 0.4_x);
        setChassisSpeed(1_x, 1_x);
        delay(200_ms);
        moveToX(3_ft - 2_in, -5_in, 1_ft + 3_in, 0.3_x);
        turnToAngle(180_deg, 0.4_x);
        odomDistanceLeft.correctPosition();
        moveToPoint(3_ft + 2_in, -9_in, true, 0.4_x, 0.7_x);
        setChassisSpeed(0.3_x, 0.3_x);
        delay(MATCHLOAD_THREE_BLOCKS);
        odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);

        // Score 7 balls in right long goal
        moveToPoint(3_ft + 3_in, 1_ft + 3_in, false, 0.8_x);
        scoreBallsUp();
        ALIGN_TO_LONG_GOAL(1_s)
        pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
        ALIGN_TO_LONG_GOAL(0.75_s)

        // Hook descore arm into goal to secure control bonus
        hookDescoreWing(3_ft, 20_in);
    }
}

// Right Side AWP Assist Autonomous Route -----------------
void rightAwp(bool nineBall) {
    if (nineBall) {
        // 9 balls split 4/5 Long/Middle
        // Initialize starting position
        setChassisPose(3.5_in, 0.5_in, 0_deg);
        odomDistanceForward.setOriginPosition(1_ft, -4_ft);
        odomDistanceLeft.setOriginPosition(1_ft, -4_ft);

        /*
        // Grab 3 balls by top middle goal
        storeBalls();
        pros::c::adi_digital_write(kPneumaticDescorePort, HIGH);
        moveToPoint(1_ft + 2_in, 2_ft + 2.5_in, true, 0.5_x, 0.8_x, 1.5_s);

        // Grab 2 blocks from under left long goal
        moveToPose(3_ft - 2_in, 3_ft + 7_in, 60_deg, true, 0.2, 0.4_x, 0.8_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        delay(100_ms);
        
        // Score 4 balls into left long goal
        setChassisSpeed(-1_x, 0.4_x);
        delay(400_ms);
        swingToAngle(0_deg, lemlib::DriveSide::RIGHT, 0.8_x);
        moveToPoint(2_ft + 9_in, 2_ft - 2_in, false, 0.7_x);
        swingToAngle(-160_deg, lemlib::DriveSide::RIGHT, 1_x);
        swingToAngle(-175_deg, lemlib::DriveSide::RIGHT, 1_x);
        scoreBallsUp();
        ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)
        */

        // Grab 3 balls by bottom middle goal
        storeBalls();
        moveToPose(1_ft - 2.5_in, 2_ft - 2_in, 10_deg, true, 0.5, 0.6_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        
        // Score 4 balls into left long goal
        turnToAngle(110_deg, 0.4_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
        setChassisSpeed(1_x, 1_x);
        delay(250_ms);
        moveToX(3_ft - 0.5_in, -3_in, 9_in, 0.4_x, 1_x, 1.5_s);
        turnToAngle(175_deg, 0.4_x);
        // odomDistanceLeft.correctPosition();
        moveToPoint(3_ft + 1_in, 15_in, false, 0.8_x);
        ALIGN_TO_LONG_GOAL(250_ms)
        scoreBallsUp();
        ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)

        // Correct position and get into right matchloader
        setChassisPose(3_ft, 2_ft);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        moveToPose(3_ft, -11.5_in, 180_deg, true, 0.4, 0.4_x, 0.6_x, 1.5_s, 0_in, true);
        scoreBallsDown();
        delay(200_ms);
        storeBalls();
        chassis.waitUntilDone();
    } else {
        // 7 balls split 4/3 Long/Middle
        // Run 4 ball right
        right(true);

        // Wait until as late as possible to leave the long goal
        while (getTime() < 11.5_s) {
            delay(20_ms);
        }

        // Exit all motions if we're still trying to wing
        chassis.cancelMotion();

        // Correct position and get into right matchloader
        odomDistanceForward.ignoreTooLargeCorrections = true;
        odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
        odomDistanceForward.ignoreTooLargeCorrections = false;
        pros::c::adi_digital_write(kPneumaticDescorePort, HIGH);
        moveToPoint(4_ft - 3_in, 2_ft - 3_in, true, 0.5_x);
        turnToAngle(-150_deg, 0.4_x);
        pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
        setMotorSpeed(intakeBottomLarge, 1_x);  // Only put balls at the bottom of the intake
        moveToPose(3_ft - 1_in, -14.5_in, 180_deg, true, 0.4, 0.5_x, 1_x, 1.5_s);
    }

    // Unload 3 blocks from right matchloader
    setChassisSpeed(0.2_x, 0.2_x);
    delay(MATCHLOAD_THREE_BLOCKS);
    odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
    odomDistanceLeft.correctPosition(OdomDistance::ForceToWall::LEFT_RIGHT);

    // Score 3 blocks in bottom middle goal
    setMotorSpeed(intakeBottomSmall, -0.7_x);  // Stack up blocks at the bottom of the intake
    relative(-4_in, 0.4_x);
    turnToAngle(-60_deg, 0.3_x);
    pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
    moveToPose(0_in, 3_ft - 3_in, -45_deg, true, 0.5, 0.7_x, 1_x, 2_s);
    pros::c::adi_digital_write(kPneumaticDescorePort, LOW);
    scoreBallsDown();
    if (nineBall) {
        setMotorSpeed(intakeBottomLarge, -0.7_x);
    }
    delay(5_s);
}

// Solo AWP Autonomous Route ------------------------------
void soloAwp() {
    // Counter solo sig awp split 4/4/6 Long/Middle/Long
    // Initialize starting position
    setChassisPose(-1_ft, 0_in, -90_deg);
    odomDistanceForward.setOriginPosition(1_ft, -4_ft);
    odomDistanceLeft.setOriginPosition(1_ft, -4_ft);
    odomDistanceRight.setOriginPosition(1_ft, -4_ft);
    odomDistanceBackwards.setOriginPosition(1_ft, -4_ft);
    storeBalls();
    relative(3_in, 0.4_x);
    delay(100_ms);

    // Unload 3 balls from right matchloader
    moveToPoint(3_ft - 2_in, 0_ft, false, 1_x, 1_x, 0.5_s);
    moveToX(3_ft - 5_in, -3_in, 3_in, 0.1_x, 1_x, 2_s);
    odomDistanceBackwards.correctPosition();
    // chassis.waitUntilDone();
    pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
    // turnToAngle(-175_deg, 0.4_x);
    storeBalls();
    turnToPoint(3_ft, -14_in, true, 0.3_x, 1_x, 1_s, -10_deg);
    odomDistanceLeft.correctPosition();
    moveToPoint(3_ft, -14_in, true, 0.3_x, 0.7_x);
    setChassisSpeed(0.2_x, 0.2_x);
    delay(MATCHLOAD_THREE_BLOCKS);
    odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
    odomDistanceLeft.correctPosition(OdomDistance::ForceToWall::LEFT_RIGHT);
 
    // Score 4 balls in right long goal
    moveToPoint(3_ft, 14.5_in, false, 0.6_x);
    scoreBallsUp();
    ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)
    pros::c::adi_digital_write(kPneumaticScraperPort, LOW);

    // Grab 3 balls by bottom middle goal
    setChassisPose(3_ft, 2_ft, chassisAngle());
    setChassisSpeed(1_x, -0.7_x);
    while (chassisAngle().abs() > 93_deg) {
        delay(10_ms);
    }
    storeBalls();
    moveToPoint(1_ft, 2_ft - 3_in, true, 0.8_x);

    // Grab 3 balls by top middle goal
    moveToPose(-3_ft - 2_in, 2_ft - 1.5_in, -90_deg, true, 0.4, 0.7_x, 1_x, 3_s);
    // pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
    delay(150_ms);
    odomDistanceForward.ignoreTooLargeCorrections = true;
    odomDistanceForward.correctPosition();

    // Score 4 balls in left long goal
    turnToAngle(-115_deg, 0.7_x);
    pros::c::adi_digital_write(kPneumaticDescorePort, HIGH);
    setChassisSpeed(1_x, 1_x);
    delay(250_ms);
    moveToX(-5_ft, -1_ft - 3_in, 1_ft + 9_in, 0.4_x, 1_x, 2_s);
    odomDistanceForward.correctPosition();
    odomDistanceForward.ignoreTooLargeCorrections = false;
    turnToPoint(-5_ft, 14_in, false, 0.4_x, 1_x, 1_s, -10_deg);
    moveToPoint(-5_ft, 14_in, false, 0.8_x);
    scoreBallsUp();
    ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)
    setChassisPose(-5_ft, 20_in, chassisAngle());

    // Unload 3 balls from left matchloader
    moveToPoint(-5_ft, -14.5_in, true, 0.4_x, 0.8_x, 1_s, 0_in, true);
    // Clear top section while intaking on the bottom
    setMotorSpeed(intakeBottomLarge, 1_x);
    setMotorSpeed(intakeBottomSmall, -1_x);
    setMotorSpeed(intakeTop, -1_x);
    delay(CLEAR_HOOD_SECTION);
    setMotorSpeed(intakeBottomSmall, 0_x);
    setMotorSpeed(intakeTop, 0_x);
    chassis.waitUntilDone();
    setChassisSpeed(0.35_x, 0.35_x);
    delay(MATCHLOAD_THREE_BLOCKS);
    odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
    odomDistanceRight.correctPosition(OdomDistance::ForceToWall::LEFT_RIGHT);

    // Get into top middle goal
    moveToPoint(-2_ft + 4.5_in, 3_ft, false, 0_x, 0.75_x, 0.5_s);
    // moveToPose(1.5_in, 3_ft - 0.5_in, -135_deg, false, 0.5, 1_x, 1_x, 0.5_s);
    pros::c::adi_digital_write(kPneumaticMiddleDescorePort, HIGH);
    moveToPose(-2_ft + 4.5_in, 3_ft, -135_deg, false, 0.5, 0.6_x, 1_x, 2_s);

    // Score 5 blocks in top middle goal
    pros::c::adi_digital_write(kPneumaticDescorePort, LOW);
    scoreBallsMiddle();
    delay(0.5_s);
    pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
    delay(1_s);
}

void soloAwp2() {
    // Counter solo sig awp split 4/4/6 Long/Middle/Long
    // Initialize starting position
    setChassisPose(6_in, -3.5_in, 90_deg);
    // Something might be wrong with teh distance sensor origins
    odomDistanceForward.setOriginPosition(1_ft, -4_ft);
    odomDistanceLeft.setOriginPosition(1_ft, -4_ft);
    odomDistanceRight.setOriginPosition(1_ft, -4_ft);
    odomDistanceBackwards.setOriginPosition(1_ft, -4_ft);
    // storeBalls();
    // relative(3_in, 0.4_x);
    // delay(100_ms);
    
    // Unload 3 balls from right matchloader


    moveToPoint(2_ft + 3_in, -3.5_in, true, 0.4_x, 0.7_x, 1.5_s); // Was 2_ft + 4_in originally, but something may be wrong
    pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
    odomDistanceForward.correctPosition();
    // chassis.waitUntilDone();
    turnToAngle(170_deg, 0.3_x);
    storeBalls();

    relative(6_in, 0.4_x);
    // odomDistanceRight.correctPosition();
    setChassisSpeed(0.2_x, 0.2_x);
    delay(MATCHLOAD_THREE_BLOCKS);
    odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
    odomDistanceLeft.correctPosition(OdomDistance::ForceToWall::LEFT_RIGHT);    

    // Score 4 balls in right long goal
    moveToPoint(3_ft, 2_ft - 9.5_in, false, 0.3_x, 0.6_x, 1.5_s);
    scoreBallsUp();
    ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)
    pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
    

    // Descore
    hookDescoreWing(2_ft + 9_in, 2_ft - 9.5_in);
    delay(500_ms);
    //pros::c::adi_digital_write(kPneumaticDescorePort, HIGH);
    odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
    odomDistanceLeft.correctPosition();
    relative(1_ft, 0.4_x);


    // Pick Up 3 Balls near Low Goal
    //moveToPose(2_ft + 1_in, 1_ft + 4.5_in, -45_deg, true, 0.4, 0.7_x, 1_x, 2_s);
    moveToPoint(2_ft + 4_in, 1_ft + 4.5_in, true, 0.4_x, 0.7_x, 1.5_s);
    turnToAngle(-55_deg, 0.3_x);
    // moveToPose(2_ft + 1_in, 1_ft + 4.5_in, -58_deg, true, 0.4, 0.7_x, 1_x, 2_s);
    storeBalls();
    moveToPose(6_in, 3_ft - 7_in, -45_deg, true, 0.4, 0.2_x, 0.7_x, 5_s);
    relative(6_in, 0.2_x, 0.4_x);
    moveToPose(-9_in, 3_ft + 8_in, -45_deg, true, 0.4, 0.2_x, 0.7_x, 5_s); 
    delay(500_ms);
    //moveToPoint(6_in, 2_ft + 10_in, true, 0.4_x, 0.7_x, 1.5_s);

    // Score 3 balls into Low Goal
    // moveToPoint(4_in, 3_ft - 4_in, true, 0.4_x, 0.7_x, 1.5_s);
    scoreBallsDown();
    delay(1_s);

    // Grab 3 balls by Middle Goal
    moveToPoint(1_ft - 9_in, 2_ft + 8_in, false, 0.4_x, 0.7_x, 1.5_s);
    turnToAngle(-85_deg, 0.3_x);
    //moveToPose(1_ft, 2_ft, -90_deg, false, 0.4, 0.4_x, 0.7_x, 1_s);
    odomDistanceLeft.correctPosition();
    storeBalls();
    moveToPoint(-3_ft - 3_in, 2_ft, true, 0.25_x, 0.7_x, 1.5_s);
    pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
    // odomDistanceLeft.ignoreTooLargeCorrections = true;
    // relative(3_ft + 9_in, 0.4_x);
    

    // Score 3 balls into Middle Goal
    turnToAngle(-120_deg, 0.4_x);
    moveToPose(-2_ft + 1.5_in, 3_ft + 4.5_in, -135_deg, false, 0.4, 0.4_x, 1_x, 3_s);
    //relative(1_ft + 4_in, -0.4_x);
    scoreBallsMiddle();
    delay(1_s);
    






    // moveToPose(2_ft + 4_in, -1_ft - 3.5_in, 180_deg, true, 0.4, 0.4_x, 0.7_x, 1.5_s);
    // moveToPoint(2_ft + 4_in, -1_ft + -3.5_in, true, 0.4_x, 0.7_x, 1.5_s);

    /* moveToPoint(3_ft - 2_in, 0_ft, false, 1_x, 1_x, 0.5_s);
    // moveToX(3_ft - 5_in, -3_in, 3_in, 0.1_x, 1_x, 2_s);
    // odomDistanceBackwards.correctPosition();
    // chassis.waitUntilDone();
    // pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
    // turnToAngle(-175_deg, 0.4_x);
    // storeBalls();
    turnToPoint(3_ft, -14_in, true, 0.3_x, 1_x, 1_s, -10_deg); 
    odomDistanceLeft.correctPosition();
    moveToPoint(3_ft, -14_in, true, 0.3_x, 0.7_x);
    setChassisSpeed(0.2_x, 0.2_x);
    delay(MATCHLOAD_THREE_BLOCKS);
    odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
    odomDistanceLeft.correctPosition(OdomDistance::ForceToWall::LEFT_RIGHT);
 
    // Score 4 balls in right long goal
    moveToPoint(3_ft, 14.5_in, false, 0.6_x);
    scoreBallsUp();
    ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)
    pros::c::adi_digital_write(kPneumaticScraperPort, LOW);

    // Grab 3 balls by bottom middle goal
    setChassisPose(3_ft, 2_ft, chassisAngle());
    setChassisSpeed(1_x, -0.7_x);
    while (chassisAngle().abs() > 93_deg) {
        delay(10_ms);
    }
    storeBalls();
    moveToPoint(1_ft, 2_ft - 3_in, true, 0.8_x);

    // Grab 3 balls by top middle goal
    moveToPose(-3_ft - 2_in, 2_ft - 1.5_in, -90_deg, true, 0.4, 0.7_x, 1_x, 3_s);
    // pros::c::adi_digital_write(kPneumaticScraperPort, HIGH);
    delay(150_ms);
    odomDistanceForward.ignoreTooLargeCorrections = true;
    odomDistanceForward.correctPosition();

    // Score 4 balls in left long goal
    turnToAngle(-115_deg, 0.7_x);
    pros::c::adi_digital_write(kPneumaticDescorePort, HIGH);
    setChassisSpeed(1_x, 1_x);
    delay(250_ms);
    moveToX(-5_ft, -1_ft - 3_in, 1_ft + 9_in, 0.4_x, 1_x, 2_s);
    odomDistanceForward.correctPosition();
    odomDistanceForward.ignoreTooLargeCorrections = false;
    turnToPoint(-5_ft, 14_in, false, 0.4_x, 1_x, 1_s, -10_deg);
    moveToPoint(-5_ft, 14_in, false, 0.8_x);
    scoreBallsUp();
    ALIGN_TO_LONG_GOAL(FOUR_BALLS_INTO_LONG)
    setChassisPose(-5_ft, 20_in, chassisAngle());

    // Unload 3 balls from left matchloader
    moveToPoint(-5_ft, -14.5_in, true, 0.4_x, 0.8_x, 1_s, 0_in, true);
    // Clear top section while intaking on the bottom
    setMotorSpeed(intakeBottomLarge, 1_x);
    setMotorSpeed(intakeBottomSmall, -1_x);
    setMotorSpeed(intakeTop, -1_x);
    delay(CLEAR_HOOD_SECTION);
    setMotorSpeed(intakeBottomSmall, 0_x);
    setMotorSpeed(intakeTop, 0_x);
    chassis.waitUntilDone();
    setChassisSpeed(0.35_x, 0.35_x);
    delay(MATCHLOAD_THREE_BLOCKS);
    odomDistanceForward.correctPosition(OdomDistance::ForceToWall::FRONT_BACK);
    odomDistanceRight.correctPosition(OdomDistance::ForceToWall::LEFT_RIGHT);

    // Get into top middle goal
    moveToPoint(-2_ft + 4.5_in, 3_ft, false, 0_x, 0.75_x, 0.5_s);
    // moveToPose(1.5_in, 3_ft - 0.5_in, -135_deg, false, 0.5, 1_x, 1_x, 0.5_s);
    pros::c::adi_digital_write(kPneumaticMiddleDescorePort, HIGH);
    moveToPose(-2_ft + 4.5_in, 3_ft, -135_deg, false, 0.5, 0.6_x, 1_x, 2_s);

    // Score 5 blocks in top middle goal
    pros::c::adi_digital_write(kPneumaticDescorePort, LOW);
    scoreBallsMiddle();
    delay(0.5_s);
    pros::c::adi_digital_write(kPneumaticScraperPort, LOW);
    delay(1_s); */
    
}

// Skills Route -------------------------------------------
void skills() {

}

// Misc Autons --------------------------------------------
void turnTuning() {
    setChassisPose(0_ft, 0_ft, 0_deg);
    double startAngle = chassisAngle().convert(degree);
    pros::lcd::print(3, "Start: %f", startAngle);

    turnToAngle(90_deg);
    
    double endAngle = chassisAngle().convert(degree);
    pros::lcd::print(4, "End: %f", endAngle);
    pros::lcd::print(5, "Difference: %f", endAngle - startAngle - 90);
    delay(2_s);

    turnToAngle(0_deg);
}

void translationTuning() {
    setChassisPose(0_ft, 0_ft, 0_deg);
    double startY = chassis.getPose().y / 12;
    pros::lcd::print(3, "Start: %f", startY);

    relative(2_ft);
    
    double endY = chassis.getPose().y / 12;
    pros::lcd::print(4, "End: %f", endY);
    pros::lcd::print(5, "Difference: %f", endY - startY - 2);
    delay(2_s);

    // relative(-2_ft, 0_x, 0.3_x, TRANSLATE_TIMEOUT_LONG);
    moveToPoint(0_ft, 0_ft, false, 0_x, 0.3_x, 3_s);
}

void maxSpeedMeasuring() {
    double maxRecordedSpeed = 0;

    // pros::Task speedTask([&]() {
    //     while (true) {
    //         maxRecordedSpeed = std::max(maxRecordedSpeed, std::abs((float)trackingWheelFBSensor.get_velocity()) / 100.0 / 360.0 * M_PI * 2 * 2.75);
    //         delay(20_ms);
    //     }
  	// });

    relative(3_ft, 0_x, 1_x, TRANSLATE_TIMEOUT_LONG);
    pros::lcd::print(3, "Max Speed (in/sec): %f", maxRecordedSpeed);
}

void testingAuton() {
    // setChassisPose(0_ft, 0_ft, 0_deg);
    // turnToAngle(90_deg);
    // printPositionDebug();

    // odomDistanceForward.setOriginPosition(-4_ft, 3_ft);
    // odomDistanceLeft.setOriginPosition(-4_ft, 3_ft);
    // odomDistanceRight.setOriginPosition(-4_ft, -3_ft);
    odomDistanceBackwards.setOriginPosition(-4_ft, -4_ft);
    setChassisPose(0_ft, 0_ft, 0_deg);
    // turnToAngle(180_deg);
    delay(500_ms);
    // odomDistanceForward.correctPosition();
    // odomDistanceLeft.correctPosition();
    // odomDistanceRight.correctPosition();
    odomDistanceBackwards.correctPosition();

    // setChassisPose(-2_ft + 5_in, 3_ft + 7_in, -135_deg);
    // moveToX(-5_ft, -3_in, 6_in, 0_x, 1_x, 2_s);
    // printPositionDebug();

    // relative(3_in, 0.3_x);

    // setIntakeMode(SPLIT);
    // setChassisPose(0_ft, 0_ft, 0_deg);
    // score7IntoMiddle();

    // setChassisPose(0_ft, 0_ft, 180_deg);
    // moveToPose(-1.5_in, -1_ft, 180_deg, true, 0.6, 0.4_x, 1_x, 2_s);
    // moveToPose(-1.5_in, 0_ft, 180_deg, false, 0.6, 0.5_x);

    // driveStraightAtAngle(-0.5_x, 0.08_x, 180_deg, 2_s);

    // setChassisPose(0_ft, 0_ft, 180_deg);
    // moveToPose(1.5_in, -1_ft, 180_deg, true, 0.6, 0.4_x, 1_x, 2_s);
    // moveToPose(1.5_in, 0_ft, 180_deg, false, 0.6, 0.5_x);

    // driveStraightAtAngle(-0.5_x, 0.08_x, 180_deg, 2_s);
    
    // // Adjust power based on deviation from dead ahead
    // Time init = getTime();
    // Time currentTime = getTime() - init;

    // double deviationFromDesiredDegrees;
    // Speed baseSpeed;
    // Speed speedOffset;

    // while (currentTime.convert(second) < 2) {
    //     currentTime = getTime() - init;

    //     deviationFromDesiredDegrees = chassisAngle().convert(degree); // Positive -> too far right
    //     speedOffset = deviationFromDesiredDegrees * 0.08_x;
    //     baseSpeed = -0.5_x;
    //     setChassisSpeed(
    //         baseSpeed - speedOffset,
    //         baseSpeed + speedOffset
    //     );
    //     delay(20_ms);
    // }

    // setChassisSpeed(0_x, 0_x);
}
