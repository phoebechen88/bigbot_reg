#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include <math.h>
#include <string>
#include <vector>
#include "okapi/api.hpp"
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
using namespace okapi;
pros::Motor Catapult(1, false);
pros::Motor Arm(17, false);
pros::Motor Intake(13, false);
pros::Rotation RotationSensor(12);
pros::ADIDigitalOut Piston('A');
pros::Imu imu_sensor(19);

// pros::Motor Catapult();   //add port

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.







//pros::Motor Catapult();   //add port

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
 *
 *
 *
 */

// void inertialCheck(int n)
// {
// 	while(round(imu_sensor.get_rotation()) <= n)
// 	{
// 		int value = n - imu_sensor.get_rotation();
// 		bot->turnAngle(value*1_deg);
// 	}
// 	while(round(imu_sensor.get_rotation()) <= n)
// 	{		
// 		int value = imu_sensor.get_rotation() - n;
// 		bot->turnAngle(-value*1_deg);
// 		std::string text = "IMU rotation: " + std::to_string(imu_sensor.get_rotation());
// 		pros::lcd::set_text(2, text);
// 	}
// }

void autonomous()
{
	pros::Rotation RotationSensor(12);

	std::shared_ptr<ChassisController> bot = ChassisControllerBuilder()
	 .withMotors(18, -20, -9, 14) // front right and back right were reversed in order to go forward
	 // change P then D first then I only if necessary
	 // start with P I and D with zero
	 .withGains( // 0.7, 0, 0.1 results: faster, shaking less violently 0
		 // 0.5 =
		 {0.0007, 0, 0},	  // Distance controller gains
		 {0.0007, 0, 0},	  // turn controller gains
		 {0.0, 0, 0.0000} // Angle controller (helps bot drive straight)
		 )
	 .withMaxVelocity(200)
	 // Green gearset, 3 inch wheel diam, 9 inch wheel track
	 .withDimensions(AbstractMotor::gearset::green, {{5_in, 15_in}, imev5GreenTPR})
	 .build();
	pros::lcd::set_text(1, "THIS IS AUTON!");

	// // Change the code here for auton in match as the big bot will be at the other side of the field
	// bot->moveDistance(-7.9_in);
	// bot->turnAngle(70_deg);
	// bot->moveDistance(18_in);
	// bot->turnAngle(-130_deg);
	// bot->moveDistance(9.5_in);

	// Code from here is really important for the autonomous

	imu_sensor.reset();
	if (imu_sensor.is_calibrating())
	{
		pros::lcd::set_text(2, "calibrated" + std::to_string(imu_sensor.is_calibrating()));
	}
	else
	{
		pros::lcd::set_text(2, "not");

	}
	imu_sensor.tare();
	
	pros::lcd::set_text(3, "IMU initial: " + std::to_string(imu_sensor.get_yaw()));

	pros::delay(500); // pause

	bot->moveDistance(10_in);
	bot->turnAngle(20_deg);

	pros::lcd::set_text(4, "IMU post turn:" + std::to_string(imu_sensor.get_yaw()));

	pros::delay(500); // pause

	while(round(imu_sensor.get_yaw()) != 20)
	{
		if(round(imu_sensor.get_yaw()) < 20)
		{		
			int value = 20 - imu_sensor.get_yaw();
			bot->turnAngle(value*1_deg);
			// if(imu_sensor.get_rotation() < 48 || imu_sensor.get_rotation() > 52){
			// 	break;
			// }
			pros::lcd::set_text(5, "IMU fix:" + std::to_string(imu_sensor.get_yaw()));
			pros::delay(500); // pause
		}
		else if (round(imu_sensor.get_yaw()) > 20)
		{		
			int value = imu_sensor.get_yaw() - 20;
			bot->turnAngle(-value*1_deg);
			// if(imu_sensor.get_rotation() < 48 || imu_sensor.get_rotation() > 52){
			// 	break;
			// }
			pros::lcd::set_text(5, "IMU fix:" + std::to_string(imu_sensor.get_yaw()));
			pros::delay(500); // pause
		}
	}

	// while(imu_sensor.get_rotation() >= -50.0)
	// {
	// 	int value = 50 - abs(imu_sensor.get_rotation());
	// 	bot->turnAngle(-value*1_deg)
	// 	std::string text = "IMU rotation: " + std::to_string(imu_sensor.get_rotation());
	// 	pros::lcd::set_text(0, text.c_str());
	// 	if(imu_sensor.get_rotation() <= -45 && imu_sensor.get_rotation() >= -55)
	// 	{
	// 		break;
	// 	}
	// }

	bot->moveDistance(20_in);
	while (true) // Run Until the end of the autonomous period
	{
		if(RotationSensor.get_angle() < 33998) // If the rotation sensor is less than 33998
		{
			Catapult.move_velocity(200); // The catapult goes down
		}
		else // If the rotation sensor is greater than 33998
		{
			Catapult.move_velocity(0); // The catapult stops
			break;
		}
	}
	
	pros::delay(500);
	bot->moveDistance(1_in);
	Arm.move_absolute(700, 300); // The Arm for intake goes down
	bot->moveDistance(1_in);
	Intake.move_velocity(-200); // The intake starts spinning
	pros::delay(50); // Wait for the intake to start spinning

	for (int i = 0; i >= -1500; i = i - 100) // Slowly (exponentially) move the arm up
	{
		Intake.move_velocity(-100);
		Arm.move_absolute(i, 200);
		pros::delay(200);
		pros::lcd::set_text(4, "Arm2:" + std::to_string(Arm.get_position()));
	}

		Catapult.move_velocity(200); // The catapult goes down
		pros::delay(200);
		Catapult.move_velocity(0); // The catapult goes down

	}
	// }

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{

	pros::Motor FrontLeft(18, false);
	pros::Motor FrontRight(20, true);
	pros::Motor BackLeft(14, true);
	pros::Motor BackRight(9, false);
	pros::Motor MidRight(15, false);
	pros::Motor MidLeft(16, false);
	pros::Motor Catapult(1, false);
	pros::Motor Arm(17, false);
	pros::Motor Intake(13, false);
	pros::Rotation RotationSensor(12);
	pros::ADIDigitalOut Piston('A');
	pros::Imu imu_sensor(19);
	// imu_sensor.reset();



	pros::lcd::set_text(1, "READY TO DRIVE");
	int yMotion;
	int xMotion;
	int value;

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	Catapult.tare_position();
	Arm.tare_position();

	//-33 Arm encoder units for intake


	imu_sensor.tare_yaw();
	imu_sensor.tare_roll();
	imu_sensor.tare_pitch();

	while (true)
	{

	pros::lcd::set_text(1, "YAW: " + std::to_string(imu_sensor.get_yaw()));
	pros::lcd::set_text(2, "PITCH: " + std::to_string(imu_sensor.get_roll()));
	pros::lcd::set_text(3, "ROLL: " + std::to_string(imu_sensor.get_pitch()));

	// pros::lcd::set_text(2, "IMU quaternion y: " + std::to_string(qt.y));
	// pros::lcd::set_text(3, "IMU quaternion z: " + std::to_string(qt.z));
	// pros::lcd::set_text(4, "IMU quaternion w: " + std::to_string(qt.w));

		// pros::lcd::set_text(1, "Arm:" + std::to_string(Arm.get_position()));
		// pros::lcd::set_text(2, "Front Right Motor:" + std::to_string(FrontRight.get_position()));
		// pros::lcd::set_text(3, "Back Left Motor:" + std::to_string(BackLeft.get_position()));
		// pros::lcd::set_text(4, "Back Right Motor:" + std::to_string(BackRight.get_position()));
		// pros::lcd::set_text(5, "Rotation Sensor: " + std::to_string(RotationSensor.get_position()));
		// pros::lcd::set_text(6, "Catapult: " + std::to_string(Catapult.get_position()));


		// driving control code

		yMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // ik this looks wrong, but it works
		xMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

		int right = -xMotion + yMotion; //-power + turn
		int left = xMotion + yMotion;	// power + turn

		FrontLeft.move(left); // Swap negatives if you want the bot to drive in the other direction
		BackLeft.move(-left);
		// MidLeft.move(left);
		BackRight.move(right);
		FrontRight.move(-right);
		// MidRight.move(right);

		if (master.get_digital(DIGITAL_R1))
		{
			RotationSensor.set_data_rate(0);
			Catapult.move_velocity(200);
			pros::lcd::set_text(5, std::to_string(RotationSensor.get_angle()));
		}
		else if (RotationSensor.get_angle() <= 33998)
		{ // 3050 ->2700(no data rate)-> 3565 (data rate)
			Catapult.move_velocity(200);
		}
		// value to launch (34000) is value due to catapult starting position not being zero
		else
		{
			// RotationSensor.set_data_rate(55);
			Catapult.move_velocity(0);
		}

		if (master.get_digital(DIGITAL_L1))
		{
			Arm.move_velocity(-112);
			pros::lcd::set_text(5, "Arm Velocity:" + std::to_string(Arm.get_actual_velocity()));
		}
		else if (master.get_digital(DIGITAL_L2))
		{
			Arm.move_velocity(112);
			pros::lcd::set_text(5, "Arm Velocity:" + std::to_string(Arm.get_actual_velocity()));
		}
		else
		{
			Arm.move_velocity(0);
		}

		if (master.get_digital(DIGITAL_R2))
		{
			Intake.move_velocity(-200);
			pros::lcd::set_text(5, "Intake Velocity:" + std::to_string(Intake.get_actual_velocity()));
		}
		else if (master.get_digital(DIGITAL_DOWN))
		{
			Intake.move_velocity(200);
			pros::lcd::set_text(5, "Intake Velocity:" + std::to_string(Intake.get_actual_velocity()));
		}

		/* 		if(master.get_digital(DIGITAL_DOWN))
				{
					Intake.move_velocity(200);
					pros::lcd::set_text(5,"Intake Velocity:" + std::to_string(Intake.get_actual_velocity()));
				}
				else {
					Intake.move_velocity(0);
				} */

		if (master.get_digital(DIGITAL_A))
		{
			Piston.set_value(false);
		}
		else
		{
			Piston.set_value(true);
		}

		pros::delay(20);
	}
}