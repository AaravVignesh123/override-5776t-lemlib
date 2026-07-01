#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "autonomous/odometry.hpp"
#include "lemlib/api.hpp"


// ----------------------------------------------------------------------------------------------------
// --- Sensors and control schemes --------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------


// Odom sensors
pros::Rotation trackingWheelLRSensor(-15);
// pros::Rotation trackingWheelFBSensor(-18);
lemlib::TrackingWheel trackingWheelLR(&trackingWheelLRSensor, lemlib::Omniwheel::NEW_2, -2.4);
// lemlib::TrackingWheel trackingWheelFB(&trackingWheelFBSensor, lemlib::Omniwheel::NEW_275, -0.4);
pros::Imu imu(17);

lemlib::OdomSensors sensors(
    nullptr, //&trackingWheelFB,
    nullptr, // no 2nd FB tracking
    &trackingWheelLR,
    nullptr, // no 2nd LR tracking
    &imu
);

// PIDs
lemlib::ControllerSettings chassisDrivePid(
    6, // kP
    0, // kI
    0.9, // kD
    3, // anti windup
    2, // small error range, in inches
    100, // small error range timeout, in milliseconds
    4, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);
lemlib::ControllerSettings chassisTurnPid(
    1.75, // kP
    0, // kI
    9.5, // kD
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);


// ----------------------------------------------------------------------------------------------------
// --- Chassis Functions and PROS Wrappers ------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------


// Chassis status functions
bool isOverheating() {
    return 
        driveLF.is_over_temp() || 
        driveLC.is_over_temp() ||
        driveLB.is_over_temp() || 
        driveRF.is_over_temp() ||
        driveRC.is_over_temp() || 
        driveRB.is_over_temp();
}

// Position setter function
void setChassisPose(Length x, Length y) {
    lemlib::Pose startingPose(x.convert(inch), y.convert(inch), chassisAngle().convert(degree));
    chassis.setPose(startingPose);
}

// Position setter function
void setChassisPose(Length x, Length y, Angle theta) {
    lemlib::Pose startingPose(x.convert(inch), y.convert(inch), theta.convert(degree));
    chassis.setPose(startingPose);
}

// Position getter functions using units library
Length chassisX() {
    return chassis.getPose().x * inch;
}
Length chassisY() {
    return chassis.getPose().y * inch;
}
Angle chassisAngle() {
    double baseAngle = chassis.getPose(true).theta;
    if (baseAngle > M_PI) baseAngle -= M_TWOPI;
    if (baseAngle < -M_PI) baseAngle += M_TWOPI;
    return baseAngle * radian;
}
Angle chassisPitch() {
    return -imu.get_roll() * degree;
}

// Motor power setting
void setMotorSpeed(pros::Motor motor, Speed speed) {
    double _speed = std::clamp(speed.convert(motorPowerScale), -127.0, 127.0);
    motor.move(_speed);
}

// Chassis power setting
void setChassisSpeed(Speed left, Speed right) {
    double _left = std::clamp(left.convert(motorPowerScale), -127.0, 127.0);
    double _right = std::clamp(right.convert(motorPowerScale), -127.0, 127.0);
    chassis.tank(_left, _right, true);
}

// Delay
void delay(Time time) {
    int _time = time.convert(millisecond);
    pros::delay(_time);
}

// Current time
Time getTime() {
    return pros::millis() * millisecond;
}

// Position logging mid-auton
Length targetX = 0_ft;
Length targetY = 0_ft;
Angle targetAngle = 0_deg;
Time autonStartTime = 0_ms;

void printPositionDebug() {
    double _targetX = targetX.convert(inch);
    double _targetY = targetY.convert(inch);
    double _targetAngle = targetAngle.convert(degree);
    double _positionX = chassis.getPose().x;
    double _positionY = chassis.getPose().y;
    double _positionAngle = chassisAngle().convert(degree);
    double _timeElapsed = (getTime() - autonStartTime).convert(second);

    pros::lcd::print(2, "Position Debug - (inch, inch, deg)");
    pros::lcd::print(3, "Target:    (%f, %f, %f)", _targetX, _targetY, _targetAngle);
    pros::lcd::print(4, "Position:  (%f, %f, %f)", _positionX, _positionY, _positionAngle);
    pros::lcd::print(5, "Deviation: (%f, %f, %f)", _targetX - _positionX, _targetY - _positionY, _targetAngle - _positionAngle);
    pros::lcd::print(6, "Time: %fs", _timeElapsed);

    stopIntake();
    setChassisSpeed(0_x, 0_x);
    delay(1_hr);
}

// Gets heading to face towards target point
Angle getAngleToTargetPoint(Length x, Length y) {
    return atan2((y - chassisY()).convert(inch), (x - chassisX()).convert(inch)) * radian;
}

// Angle between a1 and a2 (from a1 to a2, e.g. (45, 60) -> 15)
Angle angleDiff(Angle a1, Angle a2) {
    double _a1 = a1.convert(radian);
    double _a2 = a2.convert(radian);
    return (std::fmod((std::fmod((_a2 - _a1), (2 * M_PI)) + M_PI * 3), (2 * M_PI)) - M_PI) * radian;
}

// Re-constrain angle to be within (-180_deg, 180_deg)
Angle reconstrainAngle(Angle angle) {
    angle = std::fmod(angle.convert(degree), 360) * degree;
    if (angle > 180_deg) angle -= 360_deg;
    if (angle < -180_deg) angle += 360_deg;
    return angle;
}


// ----------------------------------------------------------------------------------------------------
// --- LemLib Wrapper Functions -----------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------


// Angle function wrappers

void turnToPoint(Length x, Length y, bool forwards, Speed minSpeed, Speed maxSpeed, Time time, Angle earlyExitAngle, bool async) {
    // Convert inputs to LemLib standard units
    double _x = x.convert(inch);
    double _y = y.convert(inch);
    int _minSpeed = std::clamp(minSpeed.convert(motorPowerScale), 0.0, 127.0);  // For whatever reason, LemLib turn functions use integer speed clamps
    int _maxSpeed = std::clamp(maxSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _earlyExitAngle = earlyExitAngle.convert(degree);
    double _time = time.convert(millisecond);

    // Save target pose info
    targetAngle = getAngleToTargetPoint(x, y) + (forwards ? 0_deg : 180_deg);

    chassis.turnToPoint(_x, _y, _time, {.forwards = forwards, .maxSpeed = _maxSpeed, .minSpeed = _minSpeed, .earlyExitRange = _earlyExitAngle}, async);
}

void turnToAngle(Angle heading, Speed minSpeed, Speed maxSpeed, Time time, Angle earlyExitAngle, bool async) {
    // Convert inputs to LemLib standard units
    double _heading = heading.convert(degree);
    int _minSpeed = std::clamp(minSpeed.convert(motorPowerScale), 0.0, 127.0);  // For whatever reason, LemLib turn functions use integer speed clamps
    int _maxSpeed = std::clamp(maxSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _earlyExitAngle = earlyExitAngle.convert(degree);
    double _time = time.convert(millisecond);

    // Save target pose info
    targetAngle = heading;

    chassis.turnToHeading(_heading, _time, {.maxSpeed = _maxSpeed, .minSpeed = _minSpeed, .earlyExitRange = _earlyExitAngle}, async);
}

void swingToPoint(Length x, Length y, DriveSide lockedSide, bool forwards, Speed minSpeed, Speed maxSpeed, Time time, Angle earlyExitAngle, bool async) {
    // Convert inputs to LemLib standard units
    double _x = x.convert(inch);
    double _y = y.convert(inch);
    float _minSpeed = std::clamp(minSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _maxSpeed = std::clamp(maxSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _earlyExitAngle = earlyExitAngle.convert(degree);
    double _time = time.convert(millisecond);

    // Save target pose info
    targetAngle = getAngleToTargetPoint(x, y) + (forwards ? 0_deg : 180_deg);

    chassis.swingToPoint(_x, _y, lockedSide, _time, {.forwards = forwards, .maxSpeed = _maxSpeed, .minSpeed = _minSpeed, .earlyExitRange = _earlyExitAngle}, async);
}

void swingToAngle(Angle heading, DriveSide lockedSide, Speed minSpeed, Speed maxSpeed, Time time, Angle earlyExitAngle, bool async) {
    // Convert inputs to LemLib standard units
    double _heading = heading.convert(degree);
    float _minSpeed = std::clamp(minSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _maxSpeed = std::clamp(maxSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _earlyExitAngle = earlyExitAngle.convert(degree);
    double _time = time.convert(millisecond);

    // Save target pose info
    targetAngle = heading;

    chassis.swingToHeading(_heading, lockedSide, _time, {.maxSpeed = _maxSpeed, .minSpeed = _minSpeed, .earlyExitRange = _earlyExitAngle}, async);
}


// Movement function wrappers

void moveToPose(Length x, Length y, Angle heading, bool forwards, float carrotMultiplier, Speed minSpeed, Speed maxSpeed, Time time, Length earlyExitRange, bool async) {
    // Convert inputs to LemLib standard units
    double _x = x.convert(inch);
    double _y = y.convert(inch);
    double _heading = heading.convert(degree);
    float _minSpeed = std::clamp(minSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _maxSpeed = std::clamp(maxSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _earlyExitRange = earlyExitRange.convert(inch);
    double _time = time.convert(millisecond);

    // Save target pose info
    targetX = x;
    targetY = y;
    targetAngle = heading + (forwards ? 0_deg : 180_deg);

    chassis.moveToPose(_x, _y, _heading, _time, {.forwards = forwards, .lead = carrotMultiplier, .maxSpeed = _maxSpeed, .minSpeed = _minSpeed, .earlyExitRange = _earlyExitRange}, async);
}

void moveToPoint(Length x, Length y, bool forwards, Speed minSpeed, Speed maxSpeed, Time time, Length earlyExitRange, bool async) {
    // Convert inputs to LemLib standard units
    double _x = x.convert(inch);
    double _y = y.convert(inch);
    float _minSpeed = std::clamp(minSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _maxSpeed = std::clamp(maxSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _earlyExitRange = earlyExitRange.convert(inch);
    double _time = time.convert(millisecond);

    // Save target pose info
    targetX = x;
    targetY = y;
    targetAngle = getAngleToTargetPoint(x, y) + (forwards ? 0_deg : 180_deg);

    chassis.moveToPoint(_x, _y, _time, {.forwards = forwards, .maxSpeed = _maxSpeed, .minSpeed = _minSpeed, .earlyExitRange = _earlyExitRange}, async);
}


// Custom functions

void relative(Length distance, Speed minSpeed, Speed maxSpeed, Time time, bool async) {
    // Convert inputs to LemLib standard units
    double _distance = distance.convert(inch);
    float _minSpeed = std::clamp(minSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _maxSpeed = std::clamp(maxSpeed.convert(motorPowerScale), 0.0, 127.0);
    double _time = time.convert(millisecond);

    lemlib::Pose currentPose = chassis.getPose(true);
    
    // Calculate target point position based on current position
    double _targetX = currentPose.x + _distance * sin(currentPose.theta);
    double _targetY = currentPose.y + _distance * cos(currentPose.theta);

    // Param calculation
    bool forwards = _distance >= 0;

    // Save target pose info
    targetX = _targetX * inch;
    targetY = _targetY * inch;
    targetAngle = getAngleToTargetPoint(targetX, targetY) + (forwards ? 0_deg : 180_deg);

    chassis.moveToPoint(_targetX, _targetY, _time, {.forwards = forwards, .maxSpeed = _maxSpeed, .minSpeed = _minSpeed}, async);
}

void moveToX(Length x, Length yMinimum, Length yMaximum, Speed minSpeed, Speed maxSpeed, Time time, bool async) {
    // Convert inputs to LemLib standard units
    double _x = x.convert(inch);
    double _yMinimum = yMinimum.convert(inch);
    double _yMaximum = yMaximum.convert(inch);
    float _minSpeed = std::clamp(minSpeed.convert(motorPowerScale), 0.0, 127.0);
    float _maxSpeed = std::clamp(maxSpeed.convert(motorPowerScale), 0.0, 127.0);
    double _time = time.convert(millisecond);

    // Calculate intercept point between robot heading and the line X = _x
    lemlib::Pose currentPose = chassis.getPose(true);
    double xDifference = (x - chassisX()).convert(inch); // Negative if we are too far right, positive for too far left
    double _targetY;
    
    if (sin(currentPose.theta) == 0) {
        _targetY = currentPose.y; // Safety for if sin(currentPose.theta) is 0 to avoid a division by zero
    } else {
        _targetY = currentPose.y + xDifference / sin(currentPose.theta) * cos(currentPose.theta); // No built-in cotangent
    }
    _targetY = std::clamp(_targetY, _yMinimum, _yMaximum);

    // Param calculation
    // LemLib apparently tracks chassis angle from -2pi to 2pi so this sin() is here for that
    bool forwards = (xDifference > 0) == (chassisAngle() > 0_deg); // Go forwards if the xDifference and heading are the same sign

    // Save target pose info
    targetX = x;
    targetY = _targetY * inch;
    targetAngle = getAngleToTargetPoint(x, targetY) + (forwards ? 0_deg : 180_deg);

    chassis.moveToPoint(_x, _targetY, _time, {.forwards = forwards, .maxSpeed = _maxSpeed, .minSpeed = _minSpeed}, async);
}

void driveStraightAtAngle(Speed speed, Speed speedChangePerDegreeOffset, Angle angle, Time time) {
    // Adjust power based on deviation from dead ahead
    Time init = getTime();
    Time currentTime = getTime() - init;

    double deviationFromDesiredDegrees;
    Speed speedOffset;

    while (currentTime < time) {
        currentTime = getTime() - init;

        deviationFromDesiredDegrees = angleDiff(angle, chassisAngle()).convert(degree); // Positive -> too far right
        speedOffset = deviationFromDesiredDegrees * speedChangePerDegreeOffset;
        setChassisSpeed(
            speed - speedOffset,
            speed + speedOffset
        );
        delay(20_ms);
    }

    setChassisSpeed(0_x, 0_x);
}
