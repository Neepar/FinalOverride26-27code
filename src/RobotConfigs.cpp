#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <cstddef>
#include "robotConfigs.h"

// // Motors/Sensors for functions
pros::Motor ElevatorMotor1(14); //Change these ports to match your robot
pros::Motor ElevatorMotor2(15); //Change these ports to match your robot
pros::Motor ElevatorMotor3(8); //Change these ports to match your robot
pros::Motor ScoringMotor1(9); //Change this port to match your robot
// pros::Optical ColorSensor(27); //Change this port to match your robot
// pros::Motor IntakeMotor(18); //Change this port to match your robot


//Controllers for robot
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// pros::Controller partner(pros::E_CONTROLLER_PARTNER);

pros::MotorGroup leftMotors({-5, -18}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({1, 11}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// pros::MotorGroup leftMotors({15, -17, 19}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
// pros::MotorGroup rightMotors({14, -16, -20}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.25, // 11.25 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              600, // drivetrain rpm is 600
                              8 // horizontal drift is 8 (for now)
);

// imu
pros::Imu imu(3); 
// vertical tracking wheel encoder (Rotation sensor) is reversed
pros::Rotation vertical_encoder(2); 
// // horizontal tracking wheel encoder (Rotation sensor) is reversed
pros::Rotation horizontal_encoder(-6); 
// // vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -0.375); 
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 3.525);  
// Distance sensors
pros::Distance XDistSensor(16); // Change this port to match your robot
pros::Distance YDistSensor(4); // Change this port to match your robot
// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
lemlib::ControllerSettings lateral_controller(9, // kP
                                              0,   // kI
                                              26,   // kD — set to 0 for now, we're isolating kP first
                                              0,   // anti windup
                                              1,   // small error range, inches
                                              100, // small error range timeout, ms
                                              3,   // large error range, inches
                                              500, // large error range timeout, ms
                                              127   // slew
);
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              13, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


// initialize function. Runs on program startup