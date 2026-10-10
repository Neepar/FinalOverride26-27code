#ifndef PROS_ROBOT_CONFIG_H_
#define PROS_ROBOT_CONFIG_H_
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.h"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"

// Motors for Functions
extern pros::Motor ElevatorMotor1;
extern pros::Motor ElevatorMotor2;
extern pros::Motor ElevatorMotor3;
extern pros::Motor ScoringMotor1;
extern pros::Motor IntakeMotor;
extern pros::Motor IntakeMotor2;
// extern pros::Optical ColorSensor;
extern pros::Controller master;
// extern pros::Controller partner;

//Pneumatics
extern pros::adi::DigitalOut ClawPneumatic;

//Drivetrain stuff
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern lemlib::Chassis chassis;
extern pros::Rotation vertical_encoder;
// extern lemlib::TrackingWheel vertical_tracking_wheel;
// extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern pros::Rotation horizontal_encoder;
extern pros::Imu imu;
extern pros::Distance YDistSensor;
extern pros::Distance XDistSensor;
#endif // PROS_ROBOT_CONFIG_H_