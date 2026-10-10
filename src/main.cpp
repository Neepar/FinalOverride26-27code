
#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robotConfigs.h"

// Variable to track elapsed time
// Functions with Button
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate(); // this handles IMU calibration/reset internally
	ClawPneumatic.set_value(false); // Set pneumatic to default state (closed)
	while (imu.is_calibrating()) {
		pros::delay(10);
	}
	
	
	// ElevatorMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// ElevatorMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// pros::lcd::print(4, "Brake mode hold");
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void arctanfunction (double targetX, double targetY) {
	lemlib::Pose currentPose = chassis.getPose();
    double deltaX = targetX - currentPose.x;
    double deltaY = targetY - currentPose.y;
    double angleToGoal = atan2(deltaX, deltaY) * (180.0 / M_PI);
    pros::lcd::print(0, "Angle to goal: %.2f", angleToGoal);
    pros::delay(3000); 
}

void autonomous() {
chassis.setPose(0,0,0);
ScoringMotor1.move_velocity(20);
// Toggle Begin
chassis.moveToPoint(0, 8, 800, {.maxSpeed = 200});
chassis.moveToPoint(0, -8, 800, {.forwards = false, .maxSpeed = 600});
chassis.moveToPoint(0, 8, 800, {.maxSpeed = 200});
chassis.moveToPoint(0, -8, 800, {.forwards = false, .maxSpeed = 600});
// Toggle End
chassis.waitUntilDone();
chassis.setPose(0,0,0);
// Forward Begin
chassis.moveToPoint(0, 17, 800, {.maxSpeed = 100}); //Was 13 before
// Forward End
chassis.waitUntilDone();
// Turn Left Begin
chassis.turnToHeading(90, 1000, {.maxSpeed = 100});
// Turn Left End
chassis.waitUntilDone();
chassis.setPose(0,0,0);
ClawPneumatic.set_value(true);
// Forward Begin
pros::delay(200);
chassis.moveToPoint(0, 19, 800, {.maxSpeed = 100});
// Forward End
chassis.waitUntilDone();
// Pin Drop Begin
ScoringMotor1.move_velocity(-600);
pros::delay(800);
ScoringMotor1.move_velocity(0);
chassis.cancelAllMotions();
// Pin Drop End
// chassis.waitUntilDone();
// // Elevator Begin
// ElevatorMotor1.move_velocity(-200);
// ElevatorMotor2.move_velocity(200);
// // Elevator End
// pros::delay(200);
// // Move Backard Begin
// chassis.moveToPoint(0, 0, 800, {.forwards=false, .maxSpeed = 100});
// // Move Backward End
// chassis.waitUntilDone();
// ElevatorMotor1.move_velocity(0);
// ElevatorMotor2.move_velocity(0);
// // // Turn Right Begin
// chassis.turnToHeading(44, 1000, {.maxSpeed = 100});
// // Turn Right End
// chassis.waitUntilDone();
// chassis.setPose(0,0,0);
// // Move Forward (Towards the Cup)
// chassis.moveToPoint(0, 25, 999999, {.maxSpeed = 100}); //Original was 17.5
// // Move Forward (Towards the Cup) Finish
// chassis.waitUntilDone();
// // Pin and Cup Pick up Begin
// ScoringMotor1.move_velocity(600);
// ElevatorMotor1.move_velocity(400);
// ElevatorMotor2.move_velocity(-400);
// pros::delay(2000);
// // Pin and Cup Pick up End
// //Elevator Up Start
// ElevatorMotor1.move_relative(-2160, 100);
// ElevatorMotor2.move_relative(2160, 100);
// //Elevator Up Finish
// //Turn begin
// pros::delay(1400);
// // ScoringMotor1.move_velocity(600);
// chassis.turnToHeading(-117.5, 1200, {.maxSpeed = 100});
// // Turn End
// // Reset Position
// chassis.waitUntilDone();
// chassis.setPose(0,0,0);
// // Reset Position End
// // Drive Forward Begin
// chassis.moveToPoint(0, 12, 999999, {.maxSpeed = 100});//Was 10 before
// chassis.waitUntilDone();
// // Drive Forward End
// // Drop Pin Begin
//  ScoringMotor1.move_velocity(-600);
// // Drop Pin End
// pros::delay(800);
// ScoringMotor1.move_velocity(0);
// ElevatorMotor1.move_velocity(-200);
// ElevatorMotor2.move_velocity(200);

}

//Real autonomous program template
//Toggle shift
//Manually measure some of these distances btw
	// 1. Move forward 8 inches
	// 2. Move backward 8 inches at ideal speed
	// 3. Move forward 8 inches
	// 4. Move backward 8 inches at ideal speed
	// 5. Move forward <= 24 inches
	// 6. Turn left 90 degrees 
	// 7. Go forward about <= 20 inches
	// 8. Score yellow pin 
	// Total score: 15 points + 12 point bonus = 27 points
	
//auton 2
// chassis.setPose(0,0,0);
// chassis.moveToPoint(0, 13, 800, {.maxSpeed = 200});
// chassis.moveToPoint(0, -5, 800, {.forwards = false, .maxSpeed = 600});
// chassis.moveToPoint(0, 13, 800, {.maxSpeed = 200});
// chassis.moveToPoint(0, -5, 800, {.forwards = false, .maxSpeed = 600});
// // Toggle
// chassis.waitUntilDone();
// chassis.setPose(0,0,0);
// chassis.moveToPoint(0, 13.5, 800, {.maxSpeed = 200}); //was 15 before
// chassis.waitUntilDone();
// chassis.turnToHeading(-90, 1000, {.maxSpeed = 200});//Replace with chassis.turntoPoint
// chassis.waitUntilDone();
// chassis.setPose(0,0,0);
// chassis.moveToPoint(0, 16.5, 800, {.maxSpeed = 200});
// chassis.waitUntilDone();
// ScoringMotor1.move_velocity(-600);
// pros::delay(500);
// ScoringMotor1.move_velocity(0);

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	bool HoldFunction = false;
	bool Pneumatic = false;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		chassis.arcade(dir, turn);

		// Buttons
		// Hold Mode
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {  // If B is pressed, toggle HoldFunction
			HoldFunction = !HoldFunction;
			leftMotors.set_brake_mode(HoldFunction ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
			rightMotors.set_brake_mode(HoldFunction ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
		}
		master.print(0, 0, "Hold Mode: %s", HoldFunction ? "ON" : "OFF");

	// Motorized Claw control
	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
	ScoringMotor1.move_velocity(600);
	IntakeMotor.move_velocity(600);
	IntakeMotor2.move_velocity(-600);
	}
	else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		ScoringMotor1.move_velocity(-600);
		IntakeMotor.move_velocity(-600);
		IntakeMotor2.move_velocity(600);
	}
	else {
		ScoringMotor1.move_velocity(0);
		IntakeMotor.move_velocity(0);
		IntakeMotor2.move_velocity(0);
	}
// Elevator Control

if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
	ElevatorMotor1.brake();
	ElevatorMotor2.brake();
	ElevatorMotor1.move_velocity(600);
	ElevatorMotor2.move_velocity(-600);
	// ElevatorMotor3.move_velocity(-600);
	// pros::lcd::print(3, "Elevator Up");

} 
else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
	ElevatorMotor1.brake();
	ElevatorMotor2.brake();
	ElevatorMotor1.move_velocity(-600);
	ElevatorMotor2.move_velocity(600);
	// ElevatorMotor3.move_velocity(600);
	// pros::lcd::print(4, "Elevator Down");
} else {
	 ElevatorMotor1.move_velocity(0);
	 ElevatorMotor2.move_velocity(0);
	//  ElevatorMotor3.move_velocity(0);
	//  pros::lcd::print(5, "Elevator Still");
}


// Pneumatic
if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {  // If X is pressed, toggle Pneumatic
	Pneumatic = !Pneumatic;
	ClawPneumatic.set_value(Pneumatic);
}

// 	// Color sensor for blue case
// 	double color = ColorSensor.get_hue();
// 	if (color >= 235 && color <= 275 && IntakeMotor.get_actual_velocity() > 0) {
// 	    IntakeMotor.move_velocity(-200); // reverse - reject the blue pin
// 	} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
// 	    IntakeMotor.move_velocity(200); // normal intake
// 	} else {
//     IntakeMotor.move_velocity(0); // stop
// }

	// // Color sensor for red case
	// double color = ColorSensor.get_hue();
	// if (color >= 0 && color <= 60 && IntakeMotor.get_actual_velocity() > 0) {
	//     IntakeMotor.move_velocity(-200); // reverse - reject the red pin
	// } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
	//     IntakeMotor.move_velocity(200); // normal intake
	// } else {
	//     IntakeMotor.move_velocity(0); // stop
	// }

	// Just comment color sort for no color sort

	//Elevator function
	
		pros::delay(20);
	}
}