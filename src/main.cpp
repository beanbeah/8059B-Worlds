#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//base Blue
	Motor BLU(BLUPort,E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor BLD(BLDPort,E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor BRU(BRUPort,E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor BRD(BRDPort,E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor DF(Differential,E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

	//Lift green
	Motor armLeft(armLeftPort,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);
	Motor armRight(armRightPort,E_MOTOR_GEARSET_18,false,E_MOTOR_ENCODER_DEGREES);

	//tipper red
	Motor tilt(tiltPort,E_MOTOR_GEARSET_36,false,E_MOTOR_ENCODER_DEGREES);

	//penumatic init
	ADIDigitalOut canisterL(canisterLeftPort);
	ADIDigitalOut canisterR(canisterRightPort);

	//sensor init
	ADIAnalogIn armPotentiometer(armPotentiometerPort);
	ADIAnalogIn tiltPotentiometer(tiltPotentiometerPort);
	Rotation encoderR(encdRPort);
	Rotation encoderS(encdSPort);
	encoderR.reset_position();
	encoderS.reset_position();

	// Mech tasks
	Task sensorTask(sensors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensor Task");
	Task debugTask(Debug, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");

	//temp enable odom/pp task
	Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	Task controlTask(PPControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");

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
void autonomous() {


}

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
void opcontrol() {
	Motor BLU(BLUPort);
	Motor BLD(BLDPort);
	Motor BRU(BRUPort);
	Motor BRD(BRDPort);
	Motor DF(Differential);
	Motor armLeft(armLeftPort);
	Motor armRight(armRightPort);
	Motor tilt(tiltPort);

	ADIDigitalOut canisterL(canisterLeftPort);
	ADIDigitalOut canisterR(canisterRightPort);

	BLU.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BLD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BRU.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BRD.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	DF.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	Controller master(E_CONTROLLER_MASTER);

	bool tankDrive = true;
	while(true) {
		double left, right;
		if(master.get_digital_new_press(DIGITAL_Y)) tankDrive = !tankDrive;

		if(tankDrive) {
			left = master.get_analog(ANALOG_LEFT_Y);
			right = master.get_analog(ANALOG_RIGHT_Y);
		} else {
			double power = master.get_analog(ANALOG_LEFT_Y);
			double turn = master.get_analog(ANALOG_RIGHT_X);
			left = power + turn;
			right = power - turn;
		}

		int avg = (left + right)/2;
		BLU.move(left);
		BLD.move(left);
		BRU.move(right);
		BRD.move(right);
		DF.move(avg);

		//drive control

		posPrintMaster();

		delay(5);
  	}
}
