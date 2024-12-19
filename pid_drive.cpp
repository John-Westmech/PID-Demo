#include "main.h"
#include "lemlib/api.hpp"

// Constants for the drivetrain and robot configuration
const double WHEEL_DIAMETER = 0.1; // Diameter of the wheel in meters (example: 10 cm)
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI; // Circumference in meters
const double DISTANCE_TO_TRAVEL = 100; // Distance in meters

// Define motors and drivetrain
pros::Motor leftMotor1(1, pros::E_MOTOR_GEARSET_18, false);
pros::Motor leftMotor2(2, pros::E_MOTOR_GEARSET_18, false);
pros::Motor rightMotor1(3, pros::E_MOTOR_GEARSET_18, true);
pros::Motor rightMotor2(4, pros::E_MOTOR_GEARSET_18, true);
pros::Imu imu(5);

// PID Constants
const double KP = 0.5;  // Proportional gain
const double KI = 0.01; // Integral gain
const double KD = 0.1;  // Derivative gain

// Create the drivetrain using LemLib
lemlib::Drivetrain_t drivetrain = {
    {&leftMotor1, &leftMotor2}, // Left motors
    {&rightMotor1, &rightMotor2}, // Right motors
    10.0,                        // Track width (cm)
    WHEEL_DIAMETER * 100,        // Wheel diameter (cm)
    600,                         // Motor RPM
    &imu                         // Inertial sensor
};

// PID controller configuration
lemlib::ChassisController_t chassisController = {
    &drivetrain, // Reference to drivetrain
    KP, KI, KD,  // PID constants for linear motion
    0.5, 0.01, 0.1 // PID constants for turning (if needed)
};

void driveForward(double distanceMeters, double maxSpeed) {
    // Convert meters to encoder ticks using LemLib
    double distanceTicks = (distanceMeters / WHEEL_CIRCUMFERENCE) * 360.0;

    // Use LemLib to drive forward
    lemlib::driveTo(distanceTicks, maxSpeed);
}

void opcontrol() {
    // Initialize the drivetrain and IMU
    imu.reset();
    pros::delay(2000); // Wait for the IMU to calibrate

    // Move forward 100 meters
    driveForward(DISTANCE_TO_TRAVEL, 50); // 50% max speed
}
