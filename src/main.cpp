#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	//base Blue
	Motor FL1(FL1Port,E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor FL2(FL2Port,E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor FL3(FL3Port,E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor FR1(FR1Port,E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor FR2(FR2Port,E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor FR3(FR3Port,E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor armLeft(armLeftPort,E_MOTOR_GEARSET_36,true,E_MOTOR_ENCODER_DEGREES);
	Motor armRight(armRightPort,E_MOTOR_GEARSET_36,false,E_MOTOR_ENCODER_DEGREES);
	ADIDigitalOut clamp(clampPort);
	ADIDigitalOut batch(batchPort);
	ADIDigitalOut needle(needlePort);
	ADIAnalogIn armPotentiometer(armPotentiometerPort);
	Rotation encoderR(encdRPort);
	Rotation encoderS(encdSPort);
	Rotation armRot(armRotPort);
	Imu imu(imuPort);
	encoderR.reset_position();
	encoderS.reset_position();
	armRot.reset_position();
	armRot.set_reversed(true);
	imu.reset();

	Task sensorTask(sensors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensor Task");
	Task debugTask(Debug,(void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
	Task armControlTask(armControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Control Task");
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
	double start = millis();
	double smooth = 0.75;
	setOffset(-90);
	baseTurn(-90);
	delay(5);
	Task controlTask(PPControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");
	Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");

	//get AWP
	enableBase(true,true);
	delay(250);
	baseMove(-20);
	waitPP(1000);
	baseMove(12);
	waitPP(1000);
	baseTurn(calcBaseTurn(30.5, 26, false));
	waitTurn(1200);
	toSet(true);
	setArmHeight(32);
	baseMove(10);
	waitPP(1000);
	delay(250);
	setBatchState(false);
	delay(900);
	//reset batching system (needle stuck edge case)
	setBatchState(true);
	delay(900);
	setBatchState(false);
	delay(800);
	setNeedleState(false);
	baseMove(11.7);
	waitPP(1000);

	//intake L-shape rings (spot turn)
	driverArmPos(0);

	baseMove(-12);
	waitPP(1000);
	baseTurn(180,0.1);
	waitTurn(1250);
	baseMove(-30);
	waitPP(1000);
	setMaxRPMV(300);
	baseMove(-25);
	waitPP(2000);
	baseMove(-34);
	waitPP(1500);
	/*
	Curving to intake rings
	std::vector<Node> ringCurve = {position,Node(44.2,18.8), Node(40,5.8), Node(33,7.5)};
	basePP(ringCurve,1-smooth,smooth,5); //tune lookahead up to 10
	waitPP(2500);
	setMaxRPMV(300);
	baseMove(-25);
	waitPP(2500);
	std::vector<Node> mogoCurve = {position, Node(22.6,66.1), Node(22.66,81.04)};
	setMaxRPMV(500);
	basePP(mogoCurve,1-smooth,smooth,10);
	waitPP(2500);
	*/
	// toSet(true);
	// setArmHeight(32);
	// baseMove(15);
	// waitPP(1000);
	// delay(250);
	// setBatchState(false);
	printf("Ended in %.2f seconds\n", (millis()-start)/1000);
	controlTask.remove();
	odometryTask.remove();
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
	Motor FL1(FL1Port);
  Motor FL2(FL2Port);
  Motor FL3(FL3Port);
  Motor FR1(FR1Port);
  Motor FR2(FR2Port);
  Motor FR3(FR3Port);
	Motor armLeft(armLeftPort);
	Motor armRight(armRightPort);

	FL1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FL2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FL3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FR1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FR2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FR3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);


	Controller master(E_CONTROLLER_MASTER);
	Controller partner(E_CONTROLLER_PARTNER);

	int armPos = 0, goalPos = 0, tiltPos = 0;
	bool tankDrive = true, init = true, state = false;
	int tick = 0;
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

		FL1.move(left);
		FL2.move(left);
		FL3.move(left);
		FR1.move(right);
		FR2.move(right);
		FR3.move(right);

		if (armClampState){
			if(init)master.rumble("...");
			if ((master.get_digital_new_press(DIGITAL_L1) || partner.get_digital_new_press(DIGITAL_R1)) && goalPos < 3){
				if (goalPos == 1) goalPos = 3;
				else ++goalPos;
			}
			else if ((master.get_digital_new_press(DIGITAL_L2) || partner.get_digital_new_press(DIGITAL_R2)) && goalPos > 0)--goalPos;
			if(tick%500==0)master.rumble("...");
			driverArmPos(goalPos);
		} else {
			init = false;
			if((master.get_digital_new_press(DIGITAL_L1) || partner.get_digital_new_press(DIGITAL_R1)) && armPos < 3) driverArmPos(++armPos);
			else if(master.get_digital_new_press(DIGITAL_L2) || partner.get_digital_new_press(DIGITAL_R2)){
				if (armPos > 0) {
					state = false;
					--armPos;
				}
				if (goalPos == 2){
					goalPos = 0, armPos = 0;
					state = true;
				}
				driverArmPos(armPos,state);
			}
		}

		if (armManual && !armClampState){
			armPos = getNearestPosition();
		}

		if(master.get_digital_new_press(DIGITAL_X)) toggleArmClampState();
		if(master.get_digital_new_press(DIGITAL_R1)) toggleNeedleState();
		if(master.get_digital_new_press(DIGITAL_R2)) toggleBatchState();
		if (master.get_digital_new_press(DIGITAL_B)) resetLift();

		master.print(2,0,"meong");
		tick++;
		delay(5);
  	}
}
