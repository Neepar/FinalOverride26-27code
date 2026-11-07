
#include "main.h"
#include "RobotConfigs.h"
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

float LEFT_WALL_X=0;
float BACK_WALL_Y=0;

float BACK_DISTANCE_SENSOR_OFFSET = 5.46;
float LEFT_DISTANCE_SENSOR_OFFSET = 3.7;

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
resetPoseNoWait();
chassis.moveToPose(-15, 14, -90, 1800, {.lead = 0.5});
chassis.waitUntilDone();
scorePin(600, 1000);
elevatorUp(300, 500);
chassis.moveToPoint(1.2, 14, 2000, {.forwards=false, .maxSpeed=60});
pros::delay(1000);
chassis.moveToPose(-20, 25, -30, 2000, {.lead=0.02, .maxSpeed=60}); //Was 25 before
chassis.waitUntilDone();
pickupPin(600, 600, 1500);
elevatorUp(600, 750);
chassis.turnToPoint(-24.85, 24.98, 2000);
chassis.moveToPoint(-24.8, 21.5, 2000, {.maxSpeed=60});
chassis.waitUntilDone();
scorePin(600, 1000);
elevatorUp(100, 500);
chassis.moveToPoint(-20, 40, 2000, {.forwards=false});
chassis.moveToPose(-40.93, 18.5, -132.08, 2000, {.lead=0.25, .maxSpeed=60});
chassis.waitUntilDone();
pickupPin(600, 600, 1200);
elevatorUp(600, 1500);
chassis.turnToHeading(-270, 800);
chassis.moveToPoint(-30, 18.5, 2000, {.maxSpeed=60});
chassis.waitUntilDone();
scorePin(600, 800);
elevatorUp(100, 500);



while (true) {
lemlib::Pose pose = chassis.getPose();
    pros::lcd::print(0, "X: %.2f", pose.x);
    pros::lcd::print(1, "Y: %.2f", pose.y);
    pros::lcd::print(2, "Theta: %.2f", pose.theta);
	pros::delay(20);
}

}

void DistanceSensorTracking() {
	// Get current pose from Lemlib
    lemlib::Pose current_pose = chassis.getPose();
    double current_heading = current_pose.theta;

    if (std::fabs(current_heading) < 2.0 || std::fabs(current_heading - 360.0) < 2.0) {
        
        // PROS distance sensors return millimeters. Divide by 25.4 to convert to inches.
        double left_dist_inches = LeftDistSensor.get() / 25.4;
        double back_dist_inches = BackDistSensor.get() / 25.4;

        // Keep existing pose values as defaults so we only overwrite what we successfully read
        double new_x = current_pose.x;
        double new_y = current_pose.y;
        bool updated = false;

        // Sanity check and compute X from the left sensor (ignoring glitches or out-of-range readings)
        if (left_dist_inches > 2.0 && left_dist_inches < 50.0) {
            new_x = LEFT_WALL_X + left_dist_inches + LEFT_DISTANCE_SENSOR_OFFSET;
            updated = true;
        }

        // Sanity check and compute Y from the back sensor
        if (back_dist_inches > 2.0 && back_dist_inches < 50.0) {
            new_y = BACK_WALL_Y + back_dist_inches + BACK_DISTANCE_SENSOR_OFFSET;
            updated = true;
        }

        // If a valid wall reading was made, apply the correction to Lemlib
        if (updated) {
            // Overwrite X and Y, but keep our current heading
            chassis.setPose(new_x, new_y, current_heading, false);
        }
    }
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
	
	}
	else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		ScoringMotor1.move_velocity(-600);
		
	}
	else {
		ScoringMotor1.move_velocity(0);
		
	}

	// Matchload Button
if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
	pros::lcd::print(9, "Line 9:Button X");
	clawDown();
	while (bumper_switch.get_value() == 0) {
		elevatorDown(100, 500);
	}

	
	}

	// Elevator control

if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
	ElevatorMotor1.brake();
	ElevatorMotor2.brake();
	ElevatorMotor1.move_velocity(600);
	ElevatorMotor2.move_velocity(-600);
	// ElevatorMotor3.move_velocity(-600);
//Down
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
if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {  // If X is pressed, toggle Pneumatic
	Pneumatic = !Pneumatic;
	ClawPneumatic.set_value(Pneumatic);
}

// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
// 	ToggleMotor.move_velocity(300);
// }
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

// Simplified Functions
void moveForward(float y, int timeout, float maxSpeed) {
	chassis.moveToPoint(0, y, timeout, {.maxSpeed = maxSpeed});
}

void moveBackward(float y, int timeout, float maxSpeed) {
	chassis.moveToPoint(0, -y, timeout, {.forwards = false, .maxSpeed = maxSpeed});
}
void resetPose() {
	chassis.waitUntilDone();
	chassis.setPose(0,0,0);
}
void turnLeft(float angle, int timeout, int maxSpeed) {
	chassis.turnToHeading(-angle, timeout, {.maxSpeed = maxSpeed});
}
void turnRight(float angle, int timeout, int maxSpeed) {
	chassis.turnToHeading(angle, timeout, {.maxSpeed = maxSpeed});
}
void moveHome() {
	chassis.moveToPoint(0, 0, 800, {.forwards = false, .maxSpeed = 200});
	chassis.waitUntilDone();
}
void resetPoseNoWait() {
	chassis.setPose(0,0,0);
}
void scorePin(int speed, float time) {
	ScoringMotor1.move_velocity(-speed);
	pros::delay(time);
	ScoringMotor1.move_velocity(0);
}
void pickupPin(int elevatorspeed, int clawspeed, float time) {
	ElevatorMotor1.move_velocity(elevatorspeed);
	ElevatorMotor2.move_velocity(-elevatorspeed);
	ScoringMotor1.move_velocity(clawspeed);
	pros::delay(time);
	ElevatorMotor1.move_velocity(0);
	ElevatorMotor2.move_velocity(0);
	ScoringMotor1.move_velocity(0);
}
void elevatorUp(int speed, float time) {
	ElevatorMotor1.move_velocity(-speed);
	ElevatorMotor2.move_velocity(speed);
	pros::delay(time);
	ElevatorMotor1.move_velocity(0);
	ElevatorMotor2.move_velocity(0);
}

void elevatorDown(int speed, float time) {
	ElevatorMotor1.move_velocity(speed);
	ElevatorMotor2.move_velocity(-speed);
	pros::delay(time);
	ElevatorMotor1.move_velocity(0);
	ElevatorMotor2.move_velocity(0);
}
void clawUp() {
	ClawPneumatic.set_value(true);
}
void clawDown() {
	ClawPneumatic.set_value(false);
}
void hardStop() {
	chassis.cancelAllMotions();
    ElevatorMotor1.move_velocity(0);
    ElevatorMotor2.move_velocity(0);
    ScoringMotor1.move_velocity(0);

}