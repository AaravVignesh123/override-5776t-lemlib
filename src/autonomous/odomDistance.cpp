#include "subsystems/chassis.hpp"
#include "subsystems/intake.hpp"
#include "autonomous/odometry.hpp"
#include "autonomous/odomDistance.hpp"
#include "lemlib/api.hpp"

// Helper Functions ---------------------------------------

Length rotatedVectorX(Length x, Length y, Angle angle) {
    double _x = x.convert(inch);
    double _y = y.convert(inch);
    double _theta = angle.convert(radian);

    double rotatedX = _x * cos(_theta) - _y * sin(_theta);

    return rotatedX * inch;
}

Length rotatedVectorY(Length x, Length y, Angle angle) {
    double _x = x.convert(inch);
    double _y = y.convert(inch);
    double _theta = angle.convert(radian);

    double rotatedY = _x * sin(_theta) + _y * cos(_theta);

    return rotatedY * inch;
}

void debugExit() {
    if (STOP_ON_ERROR) {
        stopIntake();
        delay(1_hr);
    }
}


// Main class functions -----------------------------------

OdomDistance::OdomDistance(int port, Length forwardBackOffset, Length rightLeftOffset)
    : m_distanceSensor(port)
    , m_distanceOffsetFB(forwardBackOffset)
    , m_distanceOffsetLR(rightLeftOffset)
{
    m_distanceAngle = 0_deg;
    m_readingOffset = 0_in;
    originXRelativeToFieldCenter = 1000_ft;
    originYRelativeToFieldCenter = 1000_ft;
    ignoreTooLargeCorrections = false;
}

OdomDistance::OdomDistance(int port, Length forwardBackOffset, Length rightLeftOffset, Angle angle)
    : m_distanceSensor(port)
    , m_distanceOffsetFB(forwardBackOffset)
    , m_distanceOffsetLR(rightLeftOffset)
    , m_distanceAngle(angle)
{
    m_readingOffset = 0_in;
    originXRelativeToFieldCenter = 1000_ft;
    originYRelativeToFieldCenter = 1000_ft;
    ignoreTooLargeCorrections = false;
}

OdomDistance::OdomDistance(int port, Length forwardBackOffset, Length rightLeftOffset, Length readingOffset)
    : m_distanceSensor(port)
    , m_distanceOffsetFB(forwardBackOffset)
    , m_distanceOffsetLR(rightLeftOffset)
    , m_readingOffset(readingOffset)
{
    m_distanceAngle = 0_deg;
    originXRelativeToFieldCenter = 1000_ft;
    originYRelativeToFieldCenter = 1000_ft;
    ignoreTooLargeCorrections = false;
}

OdomDistance::OdomDistance(int port, Length forwardBackOffset, Length rightLeftOffset, Angle angle, Length readingOffset)
    : m_distanceSensor(port)
    , m_distanceOffsetFB(forwardBackOffset)
    , m_distanceOffsetLR(rightLeftOffset)
    , m_distanceAngle(angle)
    , m_readingOffset(readingOffset)
{
    originXRelativeToFieldCenter = 1000_ft;
    originYRelativeToFieldCenter = 1000_ft;
    ignoreTooLargeCorrections = false;
}

bool OdomDistance::isValidCorrection(Length originalCoordinate, Length correctedCoordinate) {
    if (ignoreTooLargeCorrections) return true;

    Length coordinateDeviation = (originalCoordinate - correctedCoordinate);
    return coordinateDeviation.abs() < MAXIMUM_CORRECTION;
}

void OdomDistance::setOriginPosition(Length originX, Length originY) {
    originXRelativeToFieldCenter = originX;
    originYRelativeToFieldCenter = originY;
}

bool OdomDistance::isValidOriginPosition() {
    return originXRelativeToFieldCenter.abs() < 6_ft && originYRelativeToFieldCenter.abs() < 6_ft;
}

Length OdomDistance::getDistance() {
    return m_distanceSensor.get() * millimeter + m_readingOffset;
}

void OdomDistance::correctPosition(OdomDistance::ForceToWall forceToWall) {
    Length distanceReading = getDistance();

    // Invalid distance reading safety check
    if (distanceReading > 9000_mm) {
        pros::lcd::print(2, "OdomDistance - Distance sensor detecting nothing.");
        pros::lcd::print(3, "Reading: %f", distanceReading.convert(millimeter));
        pros::lcd::print(4, "Verify robot pose and intended reset position.");
        debugExit();
        return;
    }

    Length distanceRelativeXWorldspace = rotatedVectorX(m_distanceOffsetLR, m_distanceOffsetFB, -chassisAngle());
    Length distanceRelativeYWorldspace = rotatedVectorY(m_distanceOffsetLR, m_distanceOffsetFB, -chassisAngle());
    double rawFacingRadians = reconstrainAngle(chassisAngle() + m_distanceAngle).convert(radian);

    // Uninitialized or invalid origin position safety check
    if (!isValidOriginPosition()) {
        pros::lcd::print(2, "OdomDistance - Origin position is unset or out of bounds.");
        pros::lcd::print(3, "Origin: (%f, %f)", originXRelativeToFieldCenter, originYRelativeToFieldCenter);
        pros::lcd::print(4, "Set origin at the beginning of the autonomous route.");
        debugExit();
        return;
    }

    // Worldspace and field-centered distance sensor position
    Length distancePositionX = chassisX() + distanceRelativeXWorldspace + originXRelativeToFieldCenter;
    Length distancePositionY = chassisY() + distanceRelativeYWorldspace + originYRelativeToFieldCenter;

    // Convert distance reading and angle (polar coords) to x and y lengths (cartesian coords)
    Length yDistanceToWall = distanceReading * cos(rawFacingRadians);
    Length xDistanceToWall = distanceReading * sin(rawFacingRadians);

    // Worldspace and field-centered coords of the point on the wall that the distance sensor is looking at
    Length measuredXOnWalls = distancePositionX + xDistanceToWall;
    Length measuredYOnWalls = distancePositionY + yDistanceToWall;

    // Safety checks, don't measure within 1 ft of the field corners
    bool leftRightValid = (measuredYOnWalls.abs() < 60_in || forceToWall == LEFT_RIGHT) && forceToWall != FRONT_BACK;
    bool frontBackValid = (measuredXOnWalls.abs() < 60_in || forceToWall == FRONT_BACK) && forceToWall != LEFT_RIGHT;

    // Use distance values to find which position coordinate to correct, and what to correct it to
    Length trueDistancePositionX;
    Length trueDistancePositionY;
    if (leftRightValid) {
        // Reset on left/right
        trueDistancePositionX = 6_ft * (std::signbit(rawFacingRadians) ? -1 : 1) - xDistanceToWall;
        trueDistancePositionY = distancePositionY;

        // Check if the correction is too big to be correct and abort if it's unsafe
        if (!isValidCorrection(distancePositionX, trueDistancePositionX)) {
            pros::lcd::print(5, "Sensor X: %f", distancePositionX);
            pros::lcd::print(6, "Calculated X: %f", trueDistancePositionX);
            trueDistancePositionX = distancePositionX;
            pros::lcd::print(4, "OdomDistance - X Correction too large.");
            debugExit();
            return;
        }
    } else if (frontBackValid) {
        // Reset on front/back
        trueDistancePositionX = distancePositionX;
        trueDistancePositionY = (abs(rawFacingRadians) < M_PI_2 ? 6_ft : -6_ft) - yDistanceToWall;

        // Check if the correction is too big to be correct and abort if it's unsafe
        if (!isValidCorrection(distancePositionY, trueDistancePositionY)) {
            pros::lcd::print(5, "Sensor Y: %f", distancePositionY);
            pros::lcd::print(6, "Calculated Y: %f", trueDistancePositionY);
            pros::lcd::print(7, "yDistanceToWall: %f", yDistanceToWall);
            trueDistancePositionY = distancePositionY;
            pros::lcd::print(4, "OdomDistance - Y Correction too large.");
            debugExit();
            return;
        }
    } else {
        // Safety failed, not correcting
        trueDistancePositionX = distancePositionX;
        trueDistancePositionY = distancePositionY;
        pros::lcd::print(4, "OdomDistance - Unsafe measurement.");
        pros::lcd::print(5, "Measured X: %f", measuredXOnWalls);
        pros::lcd::print(6, "Measured Y: %f", measuredYOnWalls);
        debugExit();
        return;
    }

    Length trueRobotX = trueDistancePositionX - distanceRelativeXWorldspace - originXRelativeToFieldCenter;
    Length trueRobotY = trueDistancePositionY - distanceRelativeYWorldspace - originYRelativeToFieldCenter;
    setChassisPose(trueRobotX, trueRobotY);
    pros::lcd::print(3, "New position: (%f, %f)", trueRobotX.convert(inch), trueRobotY.convert(inch));
}

void OdomDistance::correctPositionUntilEnd(OdomDistance::ForceToWall forceToWall) {
    while (chassis.isInMotion()) {
        correctPosition(forceToWall);
        delay(50_ms);
    }
}


// odomDistanceForward object ------------------------------------

// Initialize distance odom correction sensors
OdomDistance odomDistanceForward(3, 9.9_holes, 9.5_holes, 0_mm);
OdomDistance odomDistanceLeft(4, -9.5_holes, -9.5_holes, -90_deg, 1_in);
OdomDistance odomDistanceRight(7, -9.5_holes, 9.5_holes, 90_deg, 1_in);
OdomDistance odomDistanceBackwards(8, -8.5_holes, 6.5_holes, 180_deg, 0.8_in);
