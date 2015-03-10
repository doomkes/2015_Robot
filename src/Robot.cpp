#include "WPILib.h"
#include "RobotMap.h"

class Robot: public IterativeRobot
{
	RobotDrive tank;	//normal drive wheels tank drive
	CANTalon lift;
	Victor fStrafe, bStrafe;	//lift and 2 strafe motors
	Victor  fStrafe2;//only for practice bot
	Servo camera;
	Joystick lStick, rStick, liftStick;
	Solenoid Cylinders, claw, leftPlow, rightPlow, testLight;	//solenoids that control strafing wheel height
	Ultrasonic ultraLeft, ultraRight;
	PowerDistributionPanel PDBoard;
	Encoder frontCode, backCode, leftCode, rightCode;
	DigitalInput clawSwitch;
	float frontVal = 0;
	float rearVal = 0;
	float leftJoyX = 0;
	float leftJoyY = 0;
	float rightJoyX = 0;
	float rightJoyY = 0;
	float liftJoy = 0;
	float pickupInch = 1;
	float current_position = 0;
	float max_speed = 10;
	float delta_time = 0;
	float last_time = 0;
	float turbo_mode = 0.7;
	bool triggeralreadyPressed = false;
	bool triggeralreadyUnpressed = true;
	bool manualControl = false;
	int switchInt = 1;
	int time = 0;
	int autoProgram = 1;
	int lift_zero = 1;
	int tote_flip = 1;
	int container_flip = 1;
	int autoLine = 1;
	int toteState = 1;


public:

	Robot():
		tank(LEFT_MOTOR, 4, RIGHT_MOTOR, 6),	//for practice robot
		//tank(LEFT_MOTOR, RIGHT_MOTOR),	//for real robot
		lift(0),
		fStrafe(FRONT_STRAFE_MOTOR),
		bStrafe(BACK_STRAFE_MOTOR_1),
		fStrafe2(5),	//only for practice bot
		camera(8),
		lStick(LTANK_JOY_USB),
		rStick(RTANK_JOY_USB),
		liftStick(LIFT_JOY_USB),
		Cylinders(CYLINDERS),
		claw(1),
		leftPlow(3),
		rightPlow(4),
		testLight(2),
		ultraLeft(13, 12),	//real bot
		ultraRight(11, 10),
		//ultraLeft(10,11),	//practice bot
		//ultraRight(12,13),
		frontCode(2,3,true,CounterBase::k4X),
		backCode(6,7,false,CounterBase::k4X),
		leftCode(0,1,true,CounterBase::k4X),
		rightCode(4,5,false,CounterBase::k4X),
		clawSwitch(9)


	{
		tank.SetExpiration(0.1);
	}

void RobotInit()
{
	CameraServer::GetInstance()->SetQuality(50);
	CameraServer::GetInstance()->StartAutomaticCapture("cam2");

	lift.SetFeedbackDevice(CANTalon::QuadEncoder);
	lift.SetSensorDirection(true);
	lift.SelectProfileSlot(1);
	lift.SetPID(4, 0.01, 0);
	lift.SetIzone(512);
	lift.SetCloseLoopRampRate(300);
	lift.SetPosition(0);
	lift.ConfigFwdLimitSwitchNormallyOpen(true);
	lift.ConfigRevLimitSwitchNormallyOpen(false);
	lift.SetControlMode(CANSpeedController::kPosition);

	SmartDashboard::PutNumber("autoValue", 1);
	SmartDashboard::PutString("autoChoose", "Yellow Tote = 1      Robot Set = 2      Center Container = 3   "
		"   Side Container = 4        2 Container on step = 5      Double Tote Double Container = 6      Tote + Container = 7");

	frontCode.SetDistancePerPulse(0.075398);
	backCode.SetDistancePerPulse(0.075398);
	leftCode.SetDistancePerPulse(0.075398);
	rightCode.SetDistancePerPulse(0.075398);
}

void AutonomousInit()
{
	switchInt = 1;
	delta_time = 0;
	time = 0;
	lift.SetPosition(0);
	current_position = 0;
	pickupInch = 0;

	autoProgram = SmartDashboard::GetNumber("autoValue", 1.000);

	leftCode.Reset();
	rightCode.Reset();
	frontCode.Reset();
	backCode.Reset();
	toteState = 1;
}

void AutonomousPeriodic()
{
	encoderControl();
	LiftZero();
	if (autoProgram == 1) YellowTote();
	if (autoProgram == 2) RobotSet();
	if (autoProgram == 3) CenterContainer();
	if (autoProgram == 4) SideContainer();
	if (autoProgram == 5) StepContainer();
	if (autoProgram == 6) DoubleToteContainer();
	if (autoProgram == 7) ToteContainer();
	if (autoProgram == 8) ToteStack();
	//printf("%f \n", delta_time);
}
void TeleopInit()
{

}

void TeleopPeriodic()
{
	encoderControl();
	LiftZero();

	static int cameraDown = 1;
	static bool manualCamera = false;
	static bool pressed = false;
	\
	if (current_position <= 8) cameraDown = -1;
	else cameraDown = 1;
	if (liftStick.GetRawButton(5)){
		camera.SetAngle(50);
	}
	else {
		if (liftStick.GetRawButton(11)&&!pressed){
			if (manualCamera == true) manualCamera = false;
			else if (manualCamera == false) manualCamera = true;
			pressed = true;
		}
		if (!liftStick.GetRawButton(11)) pressed = false;
		//printf("manual mode is %i \n",manualCamera);
		if (manualCamera == true) camera.SetAngle(camera.GetAngle() - liftStick.GetY() * 4);
		if (manualCamera == false) camera.SetAngle(((cos(27/(sqrt(279+((current_position - 10)*(current_position - 10)))))) * 180 / 3.14159 * cameraDown) + 68);
		//camera.SetAngle((-liftStick.GetY() * 90) + 90);
		//printf("fancy math = %f\n",(cos(27/(sqrt(279+((current_position - 8)*(current_position - 8)))))) * 180 / 3.14159);
	}
	ultraLeft.SetAutomaticMode(true);
	ultraRight.SetAutomaticMode(true);
	float rangeLeft = ultraLeft.GetRangeInches();
	float rangeRight = ultraRight.GetRangeInches();
	//printf("Left = %f", rangeLeft);
	//printf("      Right = %f \n", rangeRight);

	printf("Left %f Right %f Front %f Back %f \n", leftCode.GetDistance(),rightCode.GetDistance(),frontCode.GetDistance(),backCode.GetDistance());

	leftJoyX = lStick.GetX();
	leftJoyY = lStick.GetY();
	rightJoyX = rStick.GetX();
	rightJoyY = rStick.GetY();
	liftJoy = liftStick.GetRawAxis(4);

	if (liftStick.GetPOV() == 0) claw.Set(true);	//manual out
	else if (liftStick.GetPOV() == 180) claw.Set(false);	//manual in
	else if (!clawSwitch.Get()) claw.Set(true);	//limit switch pressed
	else claw.Set(false);	//not out

	if (lStick.GetRawButton(3)) {	//turb0 mode
		if (((leftJoyY-rightJoyY) <= 0.1)&&((leftJoyY-rightJoyY) >= -0.1)){		//10% range
			leftJoyY = (leftJoyY+rightJoyY)/2;
			rightJoyY = (leftJoyY+rightJoyY)/2;
		}
		turbo_mode = 1.0;
		}
	else turbo_mode = 0.7;

	if (!lStick.GetRawButton(4)){	//not in auto line up mode
		if ((lStick.GetRawButton(1)) || (rStick.GetRawButton(1))) {	//strafing
			frontVal = leftJoyX;
			rearVal = -rightJoyX;
			Cylinders.Set(true);
			tank.TankDrive(0.0 ,0.0 ,false);
		}
		else if ((lStick.GetRawButton(2)) || (rStick.GetRawButton(2))) {	//strait strafe
			Cylinders.Set(true);
			tank.TankDrive(0.0 ,0.0 ,false);
			frontVal = (leftJoyX + rightJoyX) / 2;
			rearVal = (leftJoyX + rightJoyX) / -3;
		}
		/*else if ((lStick.GetRawButton(3)) || (rStick.GetRawButton(3))) {	//tote turn strafe
			Cylinders.Set(true);
			tank.TankDrive(0.0 ,0.0 ,false);
			frontVal = (leftJoyX + rightJoyX) / 2 * 0.500;
			rearVal = (leftJoyX + rightJoyX) / 2 * 1.000;
		}*/
		else {		//non strafing
			Cylinders.Set(false);
			if (tote_flip == 1) tank.TankDrive(-(leftJoyY* leftJoyY* leftJoyY) * turbo_mode, -(rightJoyY*rightJoyY*rightJoyY) * turbo_mode, false);
			frontVal = 0.0;
			rearVal = 0.0;
		}
	}
	static int lift_multiplier = 1;
	if (liftStick.GetRawButton(7)) {
		max_speed = 20;
		lift_multiplier = 2;

	}
	if (!liftStick.GetRawButton(7)) {
		lift_multiplier = 1;
		max_speed = 10;
	}
	//motors are always "running", but only actually do anything when the values don't equal 0
	fStrafe.Set(frontVal*frontVal*frontVal);
	bStrafe.Set(rearVal*rearVal*rearVal);

	if (liftStick.GetRawButton(1))	//stage 1
		{
		pickupInch = 7;
		triggeralreadyPressed = false;
		}
	if (liftStick.GetRawButton(2))	//stage 2
		{
		pickupInch = 18.53;
		triggeralreadyPressed = false;
		}
	if (liftStick.GetRawButton(3))	//stage 3
		{
		pickupInch = 30.06;
		triggeralreadyPressed = false;
		}
	if (liftStick.GetRawButton(4))	//stage 4
		{
		pickupInch = 41.59;
		triggeralreadyPressed = false;
		}
	if (liftStick.GetRawButton(6))	//stage 5
		{
		pickupInch = 53.12;
		triggeralreadyPressed = false;
		}
	if ((liftStick.GetRawButton(8))&&(!triggeralreadyPressed)) //pickup mode
	{
		pickupInch = pickupInch - 7;
		triggeralreadyPressed = true;
		triggeralreadyUnpressed = false;
	}
	if ((!liftStick.GetRawButton(8))&&(!triggeralreadyUnpressed)) //stack mode
	{
		pickupInch = pickupInch + 7;
		triggeralreadyPressed = false;
		triggeralreadyUnpressed = true;
	}

	if ((lift.IsRevLimitSwitchClosed())&&(liftStick.GetRawAxis(3) < -0.1)) {	//manual up
		 pickupInch = pickupInch - liftStick.GetRawAxis(3) * 0.2 * lift_multiplier;
	}
	if ((pickupInch >= 0)&&(liftStick.GetRawAxis(3) > 0.1)) {
			 pickupInch = pickupInch - liftStick.GetRawAxis(3) * 0.2 * lift_multiplier;	//manual down
	}
	/*if (current_position <= 2){
		max_speed = 4;
	}
	else{
		max_speed = 10;
	}
*/
	if (rStick.GetRawButton(5)){	//tote flipper
		lift.SetP(4);
		switch (tote_flip){
			case 1:	//bring lift to correct position
				pickupInch = 13;
				if (current_position == 13) tote_flip = 2;
				break;
			case 2:	//start lowering it
				tank.TankDrive(0.0, 0.0);
				pickupInch = 7;
				if (current_position <= 10){
					tote_flip = 3;
				}
				break;
			case 3:	//drive backwards
				tank.TankDrive(-0.7, -0.7);
				if (current_position <= 7) tote_flip = 4;
				break;
			case 4:	//stop
				tank.TankDrive(0.0, 0.0);
				pickupInch = 19.1;
				if (current_position >= 19.1) tote_flip = 5;
				break;
			case 5:	//raise lift and drive forward
				tank.TankDrive(0.5, 0.5);
				pickupInch = 13;
				if (current_position <=13) tote_flip = 6;
				break;
			case 6:	//stop, lower lift
				tank.TankDrive(0.0, 0.0);
				pickupInch = 7;
				if (current_position <= 10) tote_flip = 7;
				break;
			case 7:	//pull backwards
				tank.TankDrive(-0.5, -0.5);
				if (current_position <= 7) tote_flip = 8;
				break;
			case 8:	//stop
				tank.TankDrive(0.0, 0.0);
				pickupInch = 19.1;
				tote_flip = 9;
				break;
			case 9:	//wait for button to un-press
				tank.TankDrive(0.0, 0.0);
				if (!rStick.GetRawButton(5)) tote_flip = 1;
				break;
		}
	}
	else {
		lift.SetP(4);
		tote_flip = 1;
	}
	if (rStick.GetRawButton(4)){	//container flipper
			lift.SetP(4);
			switch (container_flip){
				case 1:	//bring lift to correct position
					pickupInch = 20.25;
					if (current_position == 20.25) container_flip = 2;
					break;
				case 2:	//start lowering it
					tank.TankDrive(0.0, 0.0);
					pickupInch = 12;
					if (current_position <= 17.25){
						container_flip = 3;
					}
					break;
				case 3:	//drive backwards
					tank.TankDrive(-0.45, -0.45);
					if (current_position <= 12) container_flip = 4;
					break;
				case 4:	//stop
					tank.TankDrive(0.0, 0.0);
					pickupInch = 25;
					if (current_position >= 25) container_flip = 5;
					break;
				case 5:	//wait for button to un-press
					tank.TankDrive(0.0, 0.0);
					if (!rStick.GetRawButton(4)) container_flip = 1;
					break;
			}
		}
	else {
		lift.SetP(4);
		container_flip = 1;
	}
	//printf("case %i \n", autoLine);
		switch (autoLine){	//auto line up
			case 1:
				if (lStick.GetRawButton(4)){
					autoLine = 2;
				}
				break;
			case 2:
				if (rangeLeft >= 12){	//is too far to the left
					Cylinders.Set(true);
					frontVal = 0.4;
					rearVal = -0.3;
				}
				if (rangeRight >= 12){	//is too far to the right
					Cylinders.Set(true);
					frontVal = -0.4;
					rearVal = 0.3;
				}
				if ((rangeLeft < 12)&&(rangeRight < 12)){
					frontVal = 0;
					rearVal = 0;
					autoLine = 3;
				}
				if(!lStick.GetRawButton(4)) autoLine = 1;
				break;
			case 3:
				if (rangeLeft > rangeRight){
					Cylinders.Set(true);
					rearVal = 0.3;
				}
				if (rangeLeft < rangeRight){
					Cylinders.Set(true);
					rearVal = -0.3;
				}
				if ((rangeLeft-rangeRight < 0.5)&&(rangeLeft-rangeRight > -0.5)){
					frontVal = 0.0;
					autoLine = 4;
				}
				if(!lStick.GetRawButton(4)) autoLine = 1;
				break;
			case 4:
				Cylinders.Set(false);
				frontVal = 0;
				rearVal = 0;
				if (!lStick.GetRawButton(4)) autoLine = 1;
			}
}

//
void TestPeriodic()
{

}
void LiftZero()
{
	switch (lift_zero)	{//automatic zero encoder
			case 1:	//check for button press
				if (liftStick.GetRawButton(10)) lift_zero = 2;
				break;
			case 2:	//bring lift down until it hits the limit switch
				pickupInch = -100;
				if (lift.IsFwdLimitSwitchClosed()) lift_zero = 3;
				break;
			case 3:	//stop it, and zero encoder
				lift.SetPosition(0.25 *  178); //178
				current_position = 0;
				pickupInch = 7;
				lift_zero = 1;
				break;
	}
}
void encoderControl()
{
	delta_time = Timer::GetFPGATimestamp() - last_time;
	if (current_position < pickupInch)
		{
			current_position = current_position + max_speed * delta_time;
		if (current_position > pickupInch)
			{
			current_position = pickupInch;
			}
		}
	else if (current_position > pickupInch)
		{
		current_position = current_position - max_speed * delta_time;
		if (current_position < pickupInch)
			{
			current_position = pickupInch;
			}
		}
	lift.Set(-current_position * 178);
	last_time = Timer::GetFPGATimestamp();
	//printf("delta time %f     current position %f      pickupInch %f      encoder Value %i \n", delta_time, current_position, pickupInch, lift.GetEncPosition());
}

void CenterContainer()	//auto
{
	switch (toteState){
			case 1:
				printf("case 1\n");
				pickupInch = 30;
				if (current_position >= 29){
					toteState = 2;
				}
				break;
			case 2:
				Cylinders.Set(true);
				frontVal = 0.6;
				rearVal = -0.35;
				if (frontCode.GetDistance()>=20){
					toteState = 3;
				}

				break;
			case 3:
				printf("case 2\n");
				frontVal = 0;
				rearVal = 0;
				Cylinders.Set(false);
				tank.TankDrive(-0.5, -0.5);
				if ((leftCode.GetDistance() <= -78)||(rightCode.GetDistance() <= -78)){
					testLight.Set(true);
					frontCode.Reset();
					toteState = 4;
				}
				break;
			case 4:
				tank.TankDrive(0.0, 0.0);
				Cylinders.Set(true);
				frontVal = -0.5;
				if (frontCode.GetDistance()<=-30){
					toteState = 5;
				}
				break;
			case 5:
				frontVal = 0;
				Cylinders.Set(false);
				lift_zero = 2;
				break;
		}

	fStrafe.Set(frontVal);
	bStrafe.Set(rearVal);

}
void RobotSet()		//auto
{
	switch (toteState)			//robot set
		{
			case 1:
				tank.TankDrive(0.5, 0.5);
				if (leftCode.GetDistance() >= 50){
					toteState = 2;
				}
				break;
			case 2:
				tank.TankDrive(0.0, 0.0);
				lift_zero = 2;
				break;
		}
}
void YellowTote()	//auto
{
	printf("Left %f Right %f Front %f Back %f \n", leftCode.GetDistance(),rightCode.GetDistance(),frontCode.GetDistance(),backCode.GetDistance());
	printf("current position%f\n",current_position);
		switch (toteState){
			case 1:
				printf("case 1\n");
				pickupInch = 6;
				tank.TankDrive(0.5, 0.5);
				if (rightCode.GetDistance() >= 138){
					testLight.Set(true);
					toteState = 2;
				}
				break;
			case 2:
				printf("case 2\n");
					tank.TankDrive(-0.5, -0.5);
					testLight.Set(false);
					if (rightCode.GetDistance() <= 138){
						toteState = 3;
					}
				break;
			case 3:
				tank.TankDrive(0.0, 0.0);
				testLight.Set(true);
				lift_zero = 2;
				break;
		}

}
void SideContainer()
{
	switch (toteState){
			case 1:
				printf("case 1\n");
				pickupInch = 30;
				if (current_position >= 29){
					toteState = 2;
				}
				break;
			case 2:
				printf("case 2\n");
				tank.TankDrive(-0.5, -0.5);
				if ((leftCode.GetDistance() <= -78)||(rightCode.GetDistance() <= -78)){
					testLight.Set(true);
					frontCode.Reset();
					toteState = 3;
				}
				break;
			case 3:
				tank.TankDrive(0.0, 0.0);
				Cylinders.Set(true);
				frontVal = -0.5;
				if (frontCode.GetDistance()<=-30){
					toteState = 4;
				}
				break;
			case 4:
				frontVal = 0;
				Cylinders.Set(false);
				lift_zero = 2;
				break;
		}

	fStrafe.Set(frontVal);
	bStrafe.Set(rearVal);

}
void StepContainer()
{

}
void DoubleToteContainer()
{
	switch(toteState){
		case 1:
			Cylinders.Set(false);
			tank.TankDrive(0.5, 0.5);
			if ((rightCode.GetDistance() >= 108)){
				testLight.Set(true);
				Cylinders.Set(true);
				toteState = 2;
			}
			break;
		case 2:
			tank.TankDrive(0.0, 0.0);
			if (time == 10){
				toteState = 3;
				testLight.Set(false);
				Cylinders.Set(false);
			}
			time ++;
			break;
		case 3:
			tank.TankDrive(-0.4, -0.4);
			if ((rightCode.GetDistance() <= 104)){
				tank.TankDrive(0.0, 0.0);
				testLight.Set(true);
				Cylinders.Set(true);
				leftPlow.Set(true);
				rightPlow.Set(true);
			}
			break;
	}
}
void ToteContainer()
{
	static float timeCount = 0;
	claw.Set(true);
	switch (toteState){
		case 1:
			tank.TankDrive(0.4, 0.4);
			if (leftCode.GetDistance() >= 12){
				toteState = 2;
			}
			break;
		case 2:
			tank.TankDrive(-0.4, -0.4);
			if (leftCode.GetDistance() <= 2){
				toteState = 3;
			}
			break;
		case 3:
			tank.TankDrive(0.0, 0.0);
			Cylinders.Set(true);
			frontVal= -0.7;
			if (frontCode.GetDistance() <= -20){
				frontVal = 0;
				Cylinders.Set(false);
				toteState = 4;
				backCode.Reset();
				leftCode.Reset();
				rightCode.Reset();
			}
			break;
		case 4:
			tank.TankDrive(0.4, 0.4);
			if (rightCode.GetDistance() >= 2){
				toteState = 5;
			}
			break;
		case 5:
			tank.TankDrive(0.0, 0.0);
			pickupInch = 30;
			if (current_position >= 29){
				toteState = 6;
			}
			break;
		case 6:
			tank.TankDrive(0.0, 0.0);
			Cylinders.Set(true);
			frontVal = 0.6;
			rearVal = 0.15;
			if (frontCode.GetDistance() >= 0){
				frontVal = 0;
				rearVal = 0;
				Cylinders.Set(false);
				toteState = 7;
			}
			break;
		case 7:
			tank.TankDrive(-0.6, -0.6);
			timeCount++;
			if (timeCount >= 50){
				toteState = 8;
				rightCode.Reset();
			}
			break;
		case 8:
			tank.TankDrive(0.5, 0.5);
			if (rightCode.GetDistance() >= 122){
				testLight.Set(true);
				toteState = 9;
			}
			break;
		case 9:
			tank.TankDrive(0.0, 0.0);
			break;
	}
	fStrafe.Set(frontVal);
	bStrafe.Set(rearVal);
}
void ToteStack()
{

	printf("Left %f Right %f Front %f Back %f \n", leftCode.GetDistance(),rightCode.GetDistance(),frontCode.GetDistance(),backCode.GetDistance());

	fStrafe.Set(frontVal);
	bStrafe.Set(rearVal);

	static int time = 0;

	switch(toteState){
		case 1:
			Cylinders.Set(true);
			max_speed = 25;
			pickupInch = 32;
			tank.TankDrive(0.0, 0.0);
			if (current_position >= 32){
				toteState = 2;
			}
			break;
		case 2:
			if (backCode.GetDistance() > -44){
				pickupInch = 32;
			}
			else pickupInch = 32 - 20 * ((-backCode.GetDistance() - 44) / 37);
			frontVal = -0.8;
			rearVal = 0.4;
			tank.TankDrive(-0.45, -0.45);
			if (backCode.GetDistance() <= -78){
				toteState = 3;
			}
			break;
		case 3:
			if (backCode.GetDistance() > -44){
				pickupInch = 34;
			}
			else pickupInch = 32 - 20 * ((-backCode.GetDistance() - 44) / 37);
			frontVal = 1.0;
			rearVal = -0.5;
			time++;
			if (time > 5){
				toteState = 4;
			}
			break;
		case 4:
			time = 0;
			pickupInch = 0;
			tank.TankDrive(0.0, 0.0);
			Cylinders.Set(false);
			frontVal = 0;
			rearVal = 0;
			leftCode.Reset();
			rightCode.Reset();
			if (current_position <= 0){
				frontCode.Reset();
				backCode.Reset();
				toteState = 5;
			}
			break;
		case 5:
			tank.TankDrive(0.5, 0.5);
			if (rightCode.GetDistance() >= 1){
				pickupInch = 32;
				toteState = 6;
			}
			break;
		case 6:
			tank.TankDrive(0.0, 0.0);
			Cylinders.Set(true);
			if (current_position > 2){
				claw.Set(true);
			}
			if (current_position >= 32){
				toteState = 7;
			}
			break;
		case 7:
			tank.TankDrive(-0.45, -0.45);
			frontVal = -0.8;
			rearVal = 0.4;
			if (backCode.GetDistance() <= -78){
				toteState = 8;
			}
			break;
		case 8:
			tank.TankDrive(-0.45, -0.45);
			frontVal = 1.0;
			rearVal = -0.5;
			time++;
			if (time > 5){
				toteState = 9;
			}
			break;
		case 9:
			tank.TankDrive(0.0, 0.0);
			frontVal = 0;
			rearVal = 0;
			break;
		/*case 1:
			tank.TankDrive(0.4, 0.4);
			if (leftCode.GetDistance() >= 4){
				toteState = 2;
			}
			break;
			*/
		/*case 2:
			tank.TankDrive(0.0, 0.0);
			pickupInch = 35;
			if (current_position >= 2){
				toteState = 3;
			}
			break;
		case 3:
			tank.TankDrive(-0.4, -0.4);
			if (leftCode.GetDistance() <= 0){
				toteState = 4;
			}
			break;

		case 4:
			tank.TankDrive(0.0, 0.0);
			break;
			*/
	}
}
void OperatorControl()	//camera stuff
{
	while (IsOperatorControl() && IsEnabled())
	{
		Wait(0.005);
	}
}
};

START_ROBOT_CLASS(Robot);
