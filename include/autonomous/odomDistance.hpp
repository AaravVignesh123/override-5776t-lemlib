#pragma once
#include "main.h"
#include "lemlib/api.hpp"


// Maximum correction a distance reset can do
#define MAXIMUM_CORRECTION 6_in

// Whether to stop the autonomous route when the correction fails
#define STOP_ON_ERROR false

// Utilize a forward-facing distance sensor to correct odometry drift by resetting on the field perimeter
class OdomDistance {
    public:
        // Create a new OdomDistance object.
        // Port - distance sensor port, 1-21
        // forwardBackOffset - forward-back Length from the center of rotation to the front of the distance sensor (+ means forward offset)
        // rightLeftOffset - right-left Length from the center of rotation to the front of the distance sensor (+ means right offset)
        OdomDistance(int port, Length forwardBackOffset, Length rightLeftOffset);

        // Create a new OdomDistance object.
        // Port - distance sensor port, 1-21
        // forwardBackOffset - forward-back Length from the center of rotation to the front of the distance sensor (+ means forward offset)
        // rightLeftOffset - right-left Length from the center of rotation to the front of the distance sensor (+ means right offset)
        // angle - angle of the distance sensor from direct ahead
        OdomDistance(int port, Length forwardBackOffset, Length rightLeftOffset, Angle angle);

        // Create a new OdomDistance object.
        // Port - distance sensor port, 1-21
        // forwardBackOffset - forward-back Length from the center of rotation to the front of the distance sensor (+ means forward offset)
        // rightLeftOffset - right-left Length from the center of rotation to the front of the distance sensor (+ means right offset)
        // readingOffset - add this distance to every distance sensor reading to account for VEX product quality assurance
        OdomDistance(int port, Length forwardBackOffset, Length rightLeftOffset, Length readingOffset);

        // Create a new OdomDistance object.
        // Port - distance sensor port, 1-21
        // forwardBackOffset - forward-back Length from the center of rotation to the front of the distance sensor (+ means forward offset)
        // rightLeftOffset - right-left Length from the center of rotation to the front of the distance sensor (+ means right offset)
        // angle - angle of the distance sensor from direct ahead
        // readingOffset - add this distance to every distance sensor reading to account for VEX product quality assurance
        OdomDistance(int port, Length forwardBackOffset, Length rightLeftOffset, Angle angle, Length readingOffset);

        // (For correctPosition and correctPositionUntilEnd)
        // Force the distance correction to assume that it is looking at a wall corresponding to the given axis
        // Bypasses safety restrictions when looking near corners, be careful.
        enum ForceToWall {
            FRONT_BACK = 'v',
            LEFT_RIGHT = 'h',
            NONE = 'n'
        };

        // Set robot (0, 0) origin position relative to field center
        void setOriginPosition(Length originX, Length originY);

        // Check if robot origin position relative to field center is valid
        bool isValidOriginPosition();

        // Get raw distance reading from the distance sensor
        Length getDistance();

        // Find robot position from the wall
        void correctPosition(OdomDistance::ForceToWall forceToWall = OdomDistance::ForceToWall::NONE);

        // Correct odom position until the motion ends at a frequency of 20 Hz
        void correctPositionUntilEnd(OdomDistance::ForceToWall forceToWall = OdomDistance::ForceToWall::NONE);

        // Set this flag to ignore the "correction too large" safety check
        bool ignoreTooLargeCorrections;

    private:
        pros::Distance m_distanceSensor;
        Length m_distanceOffsetLR;
        Length m_distanceOffsetFB;
        Angle m_distanceAngle;
        Length m_readingOffset;

        Length originXRelativeToFieldCenter;
        Length originYRelativeToFieldCenter;

        // Whether or not we're sure that the distance sensor is actually looking at a wall
        bool isValidCorrection(Length originalCoordinate, Length correctedCoordinate);
};

// Odometry correction distance sensor (forward)
extern OdomDistance odomDistanceForward;

// Odometry correction distance sensor (left)
extern OdomDistance odomDistanceLeft;

// Odometry correction distance sensor (right)
extern OdomDistance odomDistanceRight;

// Odometry correction distance sensor (backwards)
extern OdomDistance odomDistanceBackwards;
