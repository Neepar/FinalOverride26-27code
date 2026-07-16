#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "lemlib/api.hpp"
#include "robotConfigs.h"

pros::MotorGroup leftMotors({15, -17, 19}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({14, -16, -20}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.25, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 600
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(6); // change
// vertical tracking wheel encoder (Rotation sensor)
pros::Rotation vertical_encoder(9); // use whatever port your rotation sensor is on // change
// horizontal tracking wheel encoder (Rotation sensor)
pros::Rotation horizontal_encoder(13); // use whatever port your rotation sensor is on // change 
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.125); // change distance
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.75); // change distance
// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// subject to change
lemlib::ControllerSettings lateral_controller(10, // kP
                                              0, // kI
                                              3, // kD
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, ms
                                              3, // large error range, in inches
                                              500, // large error range timeout, ms
                                              0 // max acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
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
