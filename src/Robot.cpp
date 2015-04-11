#include "WPILib.h"
#include "TrapezoidalMove.h"
#include "MyIterativeRobot.h"
#include "LaserRange.h"
#include "math.h"

#define FRONT_STRAFE_FF (1.05) //bessie
#define REAR_STRAFE_FF (1)  //bessie

//function prototypes
float getFrontStrafePosition(void);
float getRearStrafePosition(void);

class Robot: public MyIterativeRobot
{
	RobotDrive tank;	//normal drive wheels tank drive
	CANTalon lift, fStrafe, bStrafe;	//lift and 2 strafe motors
	Servo camera;
	Joystick lStick, rStick, liftStick;
	Solenoid Cylinders, claw, leftPoke, capper, leftLight, rightLight;	//solenoids that control strafing wheel height
	Ultrasonic ultraLeft, ultraRight;
	Encoder leftCode, rightCode;
	DigitalInput clawSwitch;
	TrapezoidalMoveProfile aStrafeMove, landfillMove;
	LaserRange ClawRange;
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
	bool ultraLock = false;
	int switchInt = 1;
	int time = 0;
	int autoProgram = 1;
	int lift_zero = 1;
	int autoLine = 1;
	int toteState = 1;
	double magic = 53.051648;
	float position = 0;
	float autoTime = 0;
	int stepMode = 0;
	int switchState = 0;
	int auto_mode_value = 0;


public:

	Robot():
		tank(0, 4, 2, 6),
		lift(0),
		fStrafe(1),
		bStrafe(2),
		camera(8),
		lStick(0),
		rStick(1),
		liftStick(2),
		Cylinders(0),
		claw(1),
		leftPoke(2),
		capper(3),
		leftLight(4),
		rightLight(5),
		//ultraLeft(13, 12),
		ultraLeft(7, 6),
		//ultraRight(11, 10),
		ultraRight(3, 2),
		leftCode(0,1,true,CounterBase::k4X),
		rightCode(4,5,false,CounterBase::k4X),
		clawSwitch(9),
		aStrafeMove(20, 50, 1000, 82.5),
		landfillMove(20, 25, 30, 60.5),
		ClawRange()


	{
		tank.SetExpiration(0.5);
	}

void RobotInit()
{
	ultraRight.SetEnabled(true);
	//ultraLeft.SetEnabled(true);

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

	fStrafe.SetFeedbackDevice(CANTalon::QuadEncoder);
	//fStrafe.SetSensorDirection(true);
	fStrafe.SelectProfileSlot(1);
	fStrafe.SetPID(4, 0.01, 0);
	fStrafe.SetIzone(512);
	//fStrafe.SetCloseLoopRampRate(300);
	fStrafe.SetPosition(0);
	fStrafe.SetControlMode(CANSpeedController::kPercentVbus);

	bStrafe.SetFeedbackDevice(CANTalon::QuadEncoder);
	//bStrafe.SetSensorDirection(true);
	bStrafe.SelectProfileSlot(1);
	bStrafe.SetPID(4, 0.01, 0);
	bStrafe.SetIzone(512);
	//bStrafe.SetCloseLoopRampRate(300);
	bStrafe.SetPosition(0);
	bStrafe.SetControlMode(CANSpeedController::kPercentVbus);

	SmartDashboard::PutNumber("autoValue", 1);
	SmartDashboard::PutString("autoChoose", "Yellow Tote = 1      Robot Set = 2      Center Container = 3   "
		"   Side Container = 4      Tote + Container = 7      Tote Stack = 8      Landfill = 10");

	leftCode.SetDistancePerPulse(0.075398);
	rightCode.SetDistancePerPulse(0.075398);

	ClawRange.Init();

	SmartDashboard::PutNumber("Front P", 16);
	SmartDashboard::PutNumber("Front I", 0.01);
	SmartDashboard::PutNumber("Front D", 0);
	SmartDashboard::PutNumber("Back P", 4);
	SmartDashboard::PutNumber("Back I", 0);
	SmartDashboard::PutNumber("Back D", 0);
	SmartDashboard::PutNumber("Front Ramp Rate", 0);
	SmartDashboard::PutNumber("Back Ramp Rate", 0);

}
void DisabledPeriodic()
{
	fStrafe.SelectProfileSlot(1);
	bStrafe.SelectProfileSlot(1);
	lift.SelectProfileSlot(1);
	SmartDashboard::PutNumber("Laser Range", ClawRange.GetDistance());
}
void AutonomousInit()
{
	fStrafe.SetPID(SmartDashboard::GetNumber("Front P"), SmartDashboard::GetNumber("Front I"), SmartDashboard::GetNumber("Front D"));
	bStrafe.SetPID(SmartDashboard::GetNumber("Back P"), SmartDashboard::GetNumber("Back I"), SmartDashboard::GetNumber("Back D"));
	fStrafe.SetPosition(0);
	bStrafe.SetPosition(0);
	fStrafe.Set(0);
	bStrafe.Set(0);
	fStrafe.SetPID(SmartDashboard::GetNumber("Front P"), SmartDashboard::GetNumber("Front I"), SmartDashboard::GetNumber("Front D"));
	bStrafe.SetPID(SmartDashboard::GetNumber("Back P"), SmartDashboard::GetNumber("Back I"), SmartDashboard::GetNumber("Back D"));
	fStrafe.SetCloseLoopRampRate(SmartDashboard::GetNumber("Front Ramp Rate"));
	bStrafe.SetCloseLoopRampRate(SmartDashboard::GetNumber("Back Ramp Rate"));

	leftCode.Reset();
	rightCode.Reset();

	position = 0;

	switchInt = 1;
	delta_time = 0;
	last_time = Timer::GetFPGATimestamp();
	time = 0;
	lift.SetPosition(0);
	current_position = 0;
	pickupInch = 0;
	autoTime = 0;

	autoProgram = SmartDashboard::GetNumber("autoValue", 1.000);

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
	if (autoProgram == 5) ToteContainer();
	if (autoProgram == 8) ToteStack();
	if (autoProgram == 10) Landfill2();
	SmartDashboard::PutNumber("Front Distance", getFrontStrafePosition());
	SmartDashboard::PutNumber("Back Distance", getRearStrafePosition());
	SmartDashboard::PutNumber("Left Distance", leftCode.GetDistance());
	SmartDashboard::PutNumber("Right Distance", rightCode.GetDistance());
	SmartDashboard::PutNumber("Front Error", fStrafe.GetClosedLoopError());
	SmartDashboard::PutNumber("Back Error", bStrafe.GetClosedLoopError());

}
float getFrontStrafePosition(){
	return fStrafe.GetEncPosition()/magic/FRONT_STRAFE_FF;
}
float getRearStrafePosition(){
	return bStrafe.GetEncPosition()/magic/REAR_STRAFE_FF;
}
void DisabledInit(){

}
void TeleopInit()
{
	fStrafe.SetP(4);
	fStrafe.SetI(0.0);

	toteState = 0;
}

void TeleopPeriodic()
{
	SmartDashboard::PutBoolean("limit switch", clawSwitch.Get());
	SmartDashboard::PutNumber("Front Distance", fStrafe.GetEncPosition()/magic);
	SmartDashboard::PutNumber("Back Distance", bStrafe.GetEncPosition()/magic);
	SmartDashboard::PutNumber("Laser Range", ClawRange.GetDistance());
	SmartDashboard::PutNumber("Left Distance", leftCode.GetDistance());
	SmartDashboard::PutNumber("Right Distance", rightCode.GetDistance());



	encoderControl();
	LiftZero();




	//container grabber

	if (liftStick.GetRawButton(5)) capper.Set(true);
	else capper.Set(false);




	//pokers

	if (lStick.GetRawButton(4)) leftPoke.Set(true);
	else leftPoke.Set(false);




	//camera

	static int cameraDown = 1;
	static bool manualCamera = false;
	static bool pressed = false;

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




	//ultrasonic

	static float rangeLeft = 0;
	static float rangeRight = 0;
	ultraLeft.SetAutomaticMode(true);
	ultraRight.SetAutomaticMode(true);
	rangeLeft = ultraLeft.GetRangeInches();
	rangeRight = ultraRight.GetRangeInches();
	//printf("%f %f");
	SmartDashboard::PutNumber("right range", rangeRight);
	SmartDashboard::PutNumber("left range", rangeLeft);

	if (rangeRight <= 8) rightLight.Set(true);
	else rightLight.Set(false);
	if (rangeLeft <= 8) leftLight.Set(true);
	else leftLight.Set(false);

	float lVal = 0;
	float rVal = 0;
	if (rangeRight >= 12){
		rVal = -rightJoyY;
	}
	else if (rangeRight >= 8){
		rVal = -((rangeRight - 8) / 8 * rightJoyY) + 0.3;
	}
	else rVal = 0;

	if (rangeLeft >= 12){
		lVal = -leftJoyY;
	}
	else if (rangeLeft >= 8){
		lVal = -((rangeLeft - 8) / 8 * leftJoyY) + 0.3;
	}
	else rVal = 0;




	leftJoyX = lStick.GetX();
	leftJoyY = lStick.GetY();
	rightJoyX = rStick.GetX();
	rightJoyY = rStick.GetY();
	liftJoy = liftStick.GetRawAxis(4);




	//kickout

	SmartDashboard::PutNumber("pov", liftStick.GetPOV());
	if (!rStick.GetRawButton(5)){
		if (liftStick.GetPOV() == 0) claw.Set(true);	//manual out
		else if (liftStick.GetPOV() == 180) claw.Set(false);	//manual in
		else if (!clawSwitch.Get()) claw.Set(true);	//limit switch pressed
		else claw.Set(false);	//not out
	}



	//turbo

	if (lStick.GetRawButton(3)) {
		if (((leftJoyY-rightJoyY) <= 0.1)&&((leftJoyY-rightJoyY) >= -0.1)){		//10% range
			leftJoyY = (leftJoyY+rightJoyY)/2;
			rightJoyY = (leftJoyY+rightJoyY)/2;
		}
		turbo_mode = 1.0;
		}
	else turbo_mode = 0.7;




	//"tote distance lock"

	if (rStick.GetRawButton(4)) ultraLock = true;
	else ultraLock = false;




	//strafe/tankdrive mode

	static bool auto_pressed = false;

	//strafing
		if ((lStick.GetRawButton(1)) || (rStick.GetRawButton(1))) {
			fStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			bStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			frontVal = pow(leftJoyX, 2);
			rearVal = pow(rightJoyX, 2);
			if (leftJoyX < 0) frontVal = -frontVal;
			if (rightJoyX < 0) rearVal = -rearVal;
			Cylinders.Set(true);
			tank.TankDrive(0.0 ,0.0 ,false);
		}
		/*else if (lStick.GetRawButton(2)) {	//strait strafe
			fStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			bStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			Cylinders.Set(true);
			tank.TankDrive(0.0 ,0.0 ,false);
			frontVal = (leftJoyX + rightJoyX) / 2;
			rearVal = (leftJoyX + rightJoyX) / -3;
		}
		*/
		//landfill sequence
		else if (rStick.GetRawButton(5)){
			fStrafe.SetControlMode(CANSpeedController::kPosition);
			bStrafe.SetControlMode(CANSpeedController::kPosition);
			if (!auto_pressed) {
				toteState = 0;
				auto_pressed = true;
			}
			fStrafe.SetP(4);
			fStrafe.SetI(0.0);
			Landfill2();
		}
		//tote-centered strafe
		else if (lStick.GetRawButton(2)){
			fStrafe.SetControlMode(CANSpeedController::kSpeed);
			bStrafe.SetControlMode(CANSpeedController::kSpeed);
			Cylinders.Set(true);
			frontVal = (leftJoyX + rightJoyX) * 5 / 2.47 * magic;
			rearVal = (leftJoyX + rightJoyX) * 5 * magic;
		}
		//tote correction mode
		else if (rStick.GetRawButton(3)){
			fStrafe.SetControlMode(CANSpeedController::kSpeed);
			bStrafe.SetControlMode(CANSpeedController::kSpeed);
			Cylinders.Set(true);
			if (rightJoyX < -0.2){
				frontVal += 5;
				rearVal += 5;
			}
			else if (rightJoyX > 0.2){
				frontVal -= 5;
				rearVal -= 5;
			}
			else {
				frontVal = 0;
				rearVal = 0;
			}
		}
		//continuous tote stacking mode
		else if (rStick.GetRawButton(2)){
			fStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			bStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			Cylinders.Set(false);
			static float last_distance = 0;
			SmartDashboard::PutNumber("state", switchState);
			switch (switchState){
				case 0:
					leftCode.Reset();
					rightCode.Reset();
					last_distance = 0;
					pickupInch = 0;
					if (current_position <= 0){
						switchState = 1;
					}
					break;
				case 1:
					claw.Set(false);
					tank.TankDrive(0.6, 0.6);
					if ((leftCode.GetDistance() > 1) && (leftCode.GetDistance() - last_distance < 0.01)){
						pickupInch = 17;
						switchState = 2;
					}
					last_distance = leftCode.GetDistance();
					break;
				case 2:
					tank.TankDrive(0.0, 0.0);
					if
					(current_position >= 2){
						claw.Set(true);
					}
					if (current_position >= 16){
						leftCode.Reset();
						rightCode.Reset();
						switchState = 3;
					}
					break;
				case 3:
					tank.TankDrive(0.5, 0.5);
					if (leftCode.GetDistance() >= 9.5){
						switchState = 4;
					}
					break;
				case 4://stop and drop
					tank.TankDrive(0.3, 0.3);
					pickupInch = 0;
					if (current_position <= 13){
						 claw.Set(false);
					}
					if (current_position <= 0){
						switchState = 0;
						last_distance = 0;
						leftCode.Reset();
						rightCode.Reset();
					}
					break;
			}
		}
		//non strafing
		else {
			fStrafe.SetP(4);
			fStrafe.SetI(0.0);
			auto_pressed = false;
			switchState = 0;
			fStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			bStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			Cylinders.Set(false);
			if (!ultraLock) tank.TankDrive(-(leftJoyY* leftJoyY* leftJoyY) * turbo_mode, -(rightJoyY*rightJoyY*rightJoyY) * turbo_mode, false);
			else tank.TankDrive(lVal, rVal);
			//else tank.TankDrive(-leftJoyY * ((rangeLeft - 5) / 5), -rightJoyY * ((rangeRight - 5) / 5));
			SmartDashboard::PutNumber("right constraint", rightJoyY * ((rangeRight - 5) / 5));
			SmartDashboard::PutNumber("left constraint", leftJoyY * ((rangeLeft - 5) / 5));
			frontVal = 0.0;
			rearVal = 0.0;
		}

		if (!rStick.GetRawButton(5)){
			fStrafe.Set(frontVal);
			bStrafe.Set(-rearVal);
		}






	//turbo lift mode

	static int lift_multiplier = 1;

	//2X speed
	if (liftStick.GetRawButton(7)) {
		max_speed = 20;
		lift_multiplier = 2;

	}
	//4X speed
	else if (liftStick.GetRawButton(5)){
		lift_multiplier = 4;
		max_speed = 40;
	}

	//normal speed
	else {
		lift_multiplier = 1;
		max_speed = 10;
	}






	//step mode

	if (liftStick.GetRawButton(9)) stepMode = 6;
	else stepMode = 0;







	if (!rStick.GetRawButton(2) && !rStick.GetRawButton(5)){
		if (liftStick.GetRawButton(1))	//stage 1
			{
			pickupInch = 7 + stepMode;
			triggeralreadyPressed = false;
			}
		if (liftStick.GetRawButton(2))	//stage 2
			{
			pickupInch = 18.53 + stepMode;
			triggeralreadyPressed = false;
			}
		if (liftStick.GetRawButton(3))	//stage 3

			{
			pickupInch = 30.06 + stepMode;
			triggeralreadyPressed = false;
			}
		if (liftStick.GetRawButton(4))	//stage 4
			{
			pickupInch = 41.59 + stepMode;
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
	}

}

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
			case 3:	//stop it,zero encoder, and bring to position 1
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
				if (fStrafe.GetEncPosition()/magic>=20){
					toteState = 3;
				}

				break;
			case 3:
				printf("case 2\n");
				frontVal = 0;
				rearVal = 0;
				Cylinders.Set(false);
				tank.TankDrive(-0.5, -0.5);
				if ((leftCode.GetDistance() <= -78)){
					fStrafe.SetPosition(0);
					toteState = 4;
				}
				break;
			case 4:
				tank.TankDrive(0.0, 0.0);
				Cylinders.Set(true);
				frontVal = -0.5;
				if (fStrafe.GetEncPosition()/magic<=-30){
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
			case 1:	//drive until reaches the auto zone
				tank.TankDrive(0.5, 0.5);
				if (leftCode.GetDistance() >= 50){
					toteState = 2;
				}
				break;
			case 2:	//stop
				tank.TankDrive(0.0, 0.0);
				lift_zero = 2;
				break;
		}
}
void YellowTote()	//auto
{
	//printf("Left %f Right %f Front %f Back %f \n", leftCode.GetDistance(),rightCode.GetDistance(),frontCode.GetDistance(),backCode.GetDistance());
	printf("current position%f\n",current_position);
		switch (toteState){
			case 1:	//drive forward until it reaches the auto zone
				printf("case 1\n");
				pickupInch = 6;
				tank.TankDrive(0.5, 0.5);
				if (leftCode.GetDistance() >= 138){
					toteState = 2;
				}
				break;
			case 2:	//backup to make room for lift to zero
				printf("case 2\n");
					tank.TankDrive(-0.5, -0.5);
					if (leftCode.GetDistance() <= 138){
						toteState = 3;
					}
				break;
			case 3:	//zero lift
				tank.TankDrive(0.0, 0.0);
				lift_zero = 2;
				break;
		}

}
void SideContainer()
{
	switch (toteState){
			case 1:	//pick up container
				printf("case 1\n");
				pickupInch = 30;
				if (current_position >= 29){
					toteState = 2;
				}
				break;
			case 2:	//back into auto zone
				printf("case 2\n");
				tank.TankDrive(-0.5, -0.5);
				if ((leftCode.GetDistance() <= -78)){
					fStrafe.SetPosition(0);
					toteState = 3;
				}
				break;
			case 3:	//turn about 90 degrees
				tank.TankDrive(0.0, 0.0);
				Cylinders.Set(true);
				frontVal = -0.5;
				if (fStrafe.GetEncPosition()/magic<=-30){
					toteState = 4;
				}
				break;
			case 4:	//stop turning
				frontVal = 0;
				Cylinders.Set(false);
				lift_zero = 2;
				break;
		}

	fStrafe.Set(frontVal);
	bStrafe.Set(rearVal);

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
			if (fStrafe.GetEncPosition()/magic <= -20){
				frontVal = 0;
				Cylinders.Set(false);
				toteState = 4;
				leftCode.Reset();
			}
			break;
		case 4:
			tank.TankDrive(0.4, 0.4);
			if (leftCode.GetDistance() >= 2){
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
			if (fStrafe.GetEncPosition()/magic >= 0){
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
				leftCode.Reset();
			}
			break;
		case 8:
			tank.TankDrive(0.5, 0.5);
			if (leftCode.GetDistance() >= 122){
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
	static float startTime = Timer::GetFPGATimestamp();
	fStrafe.SetControlMode(CANSpeedController::kPosition);
	bStrafe.SetControlMode(CANSpeedController::kPosition);
	//printf("Left %f Right %f Front %f Back %f \n", leftCode.GetDistance(),rightCode.GetDistance(),frontCode.GetDistance(),backCode.GetDistance());

	static int time = 0;

	switch(toteState){
		case 1:
			claw.Set(false);
			max_speed = 25;
			pickupInch = 32;
			if (current_position > 2){
				tank.TankDrive(-0.55, -0.5);
			}
			if (current_position >= 20){
				Cylinders.Set(true);
			}
			if (current_position >= 32){
				toteState = 2;
			}
			break;
		case 2:
			if (bStrafe.GetEncPosition()/magic < 44){
				pickupInch = 32;
			}
			else pickupInch = 32 - 18 * ((bStrafe.GetEncPosition()/magic - 44) / 37);
			fStrafe.Set(-aStrafeMove.Position(autoTime) * magic * 84 / 81);// * 81.5 / 77);
			bStrafe.Set(aStrafeMove.Position(autoTime) * magic);
			tank.TankDrive(-0.4, -0.3);
			autoTime += 0.02;
			if (bStrafe.GetEncPosition()/magic >= 80 && aStrafeMove.Position(autoTime) * magic > 80.4){
				fStrafe.SetControlMode(CANSpeedController::kPercentVbus);
				bStrafe.SetControlMode(CANSpeedController::kPercentVbus);
				fStrafe.Set(0);
				bStrafe.Set(0);
				toteState = 3;
			}
			break;
		case 3:
			autoTime = 0;
			pickupInch = -1;
			tank.TankDrive(0.0, 0.0);
			Cylinders.Set(false);
			leftCode.Reset();
			if (current_position <= -1){
				fStrafe.SetPosition(0);
				bStrafe.SetPosition(0);
				fStrafe.Set(0);
				bStrafe.Set(0);
				toteState = 4;

			}
			break;
		case 4:
			//tank.TankDrive(0.5, 0.5);
			tank.TankDrive(0.45, 0.45);	//nessie
			if (leftCode.GetDistance() >=2.5){
				max_speed = 10;
				pickupInch = 32;
			}
			if (leftCode.GetDistance() >=5){	// neesie 5
				//tank.TankDrive(0.4, 0.4);
			toteState = 5;
			}
			break;
		case 5:
			tank.TankDrive(0.0, 0.0);
			if (current_position > 4){
				claw.Set(true);
				max_speed = 20;
				//tank.TankDrive(-0.55, -0.5);
				tank.TankDrive(-0.45, -0.4);	//nessie
			}
			if (current_position > 20){
				Cylinders.Set(true);
				max_speed = 15;
			}
			if (current_position >= 32){
				toteState = 6;
			}
			break;
		case 6:
			tank.TankDrive(-0.4, -0.3);
			fStrafe.SetControlMode(CANSpeedController::kPosition);
			bStrafe.SetControlMode(CANSpeedController::kPosition);
			fStrafe.Set(-aStrafeMove.Position(autoTime) * magic);// * 81.5 / 77);
			bStrafe.Set(aStrafeMove.Position(autoTime) * magic);
			autoTime += 0.02;
			if (bStrafe.GetEncPosition()/magic >= 80 && aStrafeMove.Position(autoTime) * magic > 80.4){
				fStrafe.SetControlMode(CANSpeedController::kPercentVbus);
				bStrafe.SetControlMode(CANSpeedController::kPercentVbus);
				fStrafe.Set(0);
				bStrafe.Set(0);
				toteState = 7;
				leftCode.Reset();
			}
			break;
		case 7:
			tank.TankDrive(0.70, 0.70);
			Cylinders.Set(false);
			frontVal = 0;
			rearVal = 0;
			pickupInch = 15;
			if (leftCode.GetDistance() < 87){
				tank.TankDrive(0.7, 0.7);
			}
			if (leftCode.GetDistance() >= 87){
				tank.TankDrive(0.5, 0.5);
			}
			if (leftCode.GetDistance() >= 107){
				time = 0;
				toteState = 8;
			}
			break;
		case 8:
			tank.TankDrive(0.0, 0.0);
			time++;
			if (time > 20){
				leftCode.Reset();
				toteState = 9;
			}
			break;
		case 9:
			{
			static float peak=-99;
			float local_dist =0;
			local_dist = leftCode.GetDistance();
			if (local_dist > peak) peak = local_dist;
			tank.TankDrive(-0.4, -0.4);	//nessie
			//tank.TankDrive(-0.5, -0.5);
			if ((local_dist - peak) < -2){
				claw.Set(false);
				max_speed = 15;
				pickupInch = 11;
				SmartDashboard::PutNumber("time", Timer::GetFPGATimestamp() - startTime);
				toteState = 10;
			}
			}
			break;
		case 10:
			tank.TankDrive(0.0, 0.0);
			pickupInch = 11;
			if (current_position <= 11){
				toteState = 11;
			}
			break;
		case 11:
			tank.TankDrive(-0.5, -0.5);
			if (leftCode.GetDistance() <= -5){
				toteState = 12;
			}
			break;
		case 12:
			tank.TankDrive(0.0, 0.0);
			break;
	}
	SmartDashboard::PutNumber("test", pickupInch);
}
void PID()
{
	fStrafe.SetControlMode(CANSpeedController::kPosition);
	bStrafe.SetControlMode(CANSpeedController::kPosition);
	Cylinders.Set(true);
	fStrafe.Set(-aStrafeMove.Position(autoTime) * magic * 81 / 77);
	bStrafe.Set(aStrafeMove.Position(autoTime) * magic);
	tank.TankDrive(-0.4, -0.3);
	autoTime += 0.02;
}
void Landfill()
{
	static float time = 0;
	fStrafe.SetControlMode(CANSpeedController::kPercentVbus);
	bStrafe.SetControlMode(CANSpeedController::kPercentVbus);
	max_speed = 20;
	//toteState = 0;
	switch(toteState){

		case 1:
			claw.Set(false);
			tank.TankDrive(0.55, 0.5);//drive forward to first tote
			if (leftCode.GetDistance() >= 2){
				pickupInch = 16;
				toteState = 2;
			}
			break;
		case 2://stop when its far enough and pick up tote
			tank.TankDrive(0.0, 0.0);
			if (current_position >= 12){
				toteState = 3;
			}
			break;
		case 3://bring tote over second tote
			tank.TankDrive(0.55, 0.5);
			if (leftCode.GetDistance() >= 9){
				toteState = 4;
			}
			break;
		case 4://stop and drop
			tank.TankDrive(0.0, 0.0);
			pickupInch = 0;
			if (current_position <= 0){
				toteState = 5;
			}
			break;
		case 5://drive to second tote and pick up
			tank.TankDrive(0.4, 0.4);
			if (leftCode.GetDistance() >= 19){
				max_speed = 10;
				pickupInch = 20;
				toteState = 6;
			}
			break;
		case 6://stop
			tank.TankDrive(0.0, 0.0);
			if (current_position >= 5){
				toteState = 7;
			}
			break;
		case 7:
			tank.TankDrive(-0.5, -0.5);
			if (leftCode.GetDistance() <= 2){
				toteState = 8;
			}
			break;
		case 8:
			tank.TankDrive(0.0, 0.0);
			Cylinders.Set(true);
			if (time > 15){
				//fStrafe.Set(0.55);
				//bStrafe.Set(-0.36);
				fStrafe.Set(landfillMove.Position(autoTime) * magic * FRONT_STRAFE_FF);
				bStrafe.Set(-landfillMove.Position(autoTime) * magic * REAR_STRAFE_FF);
				autoTime += delta_time;
			}
			time++;
			if (fStrafe.GetEncPosition() / magic >= 60){
				time = 0;
				//toteState = 9;
			}
			break;
		case 9:
			if (time >= 25){
			Cylinders.Set(false);
			tank.TankDrive(0.6, 0.6);
			}
			fStrafe.Set(0.0);
			bStrafe.Set(0.0);
			if (time >= 55){
				leftCode.Reset();
				toteState = 10;
			}
			time++;
			break;
		case 10:
			tank.TankDrive(-0.5, -0.5);
			if (leftCode.GetDistance() < -5){
				pickupInch = 0;
				toteState = 11;
			}
			break;
		case 11:
			tank.TankDrive(0.0, 0.0);
			if (current_position <= 0){
				claw.Set(false);
				toteState = 12;
			}
			break;
		case 12:
			tank.TankDrive(0.5, 0.5);
			if (leftCode.GetDistance() >= -5){
				pickupInch = 16;
				toteState = 13;
			}
			break;
		case 13:
			tank.TankDrive(0.0, 0.0);
			if (current_position >= 1){
				claw.Set(true);

			}
			if (current_position >= 16){
				toteState = 14;
			}
			break;
		case 14:
			tank.TankDrive(0.5, 0.5);
			if (leftCode.GetDistance() >= 2){
				toteState = 15;
			}
			break;
		case 15:
			tank.TankDrive(0.0, 0.0);
			pickupInch = 0;
			if (current_position <= 0){
				claw.Set(false);
				toteState = 16;
			}
			break;
		case 16:
			tank.TankDrive(-0.5,- 0.5);
			if (leftCode.GetDistance() <= -30){

				toteState = 17;
			}
			break;
		case 17:
			tank.TankDrive(0.0, 0.0);
			break;

	}

}

#define FWD_SPEED 0.45
#define FWD_HOLD_SPEED 0.2
#define FB_GAIN 0.6

void Landfill2()
{
	//fStrafe.SetPID(4, 0.01, 0);
	static float time = 0;
	static float last_distance = 0;
	float drive_balance = 1;
	fStrafe.SetControlMode(CANSpeedController::kPosition);
	bStrafe.SetControlMode(CANSpeedController::kPosition);
	max_speed = 20;
	switch(toteState){


		case 0:
			leftCode.Reset();
			rightCode.Reset();
			frontVal = 0.0;
			rearVal = 0.0;
			fStrafe.SetPosition(0);
			bStrafe.SetPosition(0);
			fStrafe.Set(0);
			bStrafe.Set(0);
			toteState = 1;
			break;
		case 1:
			claw.Set(false);
			Cylinders.Set(false);
			drive_balance = 1 + (leftCode.GetDistance() + -rightCode.GetDistance()) * FB_GAIN;
			tank.TankDrive(FWD_SPEED / drive_balance, FWD_SPEED * drive_balance);
			//tank.TankDrive(FWD_SPEED, FWD_SPEED);//drive forward to first tote
			if ((leftCode.GetDistance() > 1) && (leftCode.GetDistance() - last_distance < 0.01)){
				pickupInch = 17;
				toteState = 2;
			}
			last_distance = leftCode.GetDistance();
			break;
		case 2://stop when its far enough and pick up tote
			tank.TankDrive(0.0, 0.0);
			if (current_position >= 16){
				leftCode.Reset();
				rightCode.Reset();
				toteState = 3;
			}
			break;
		case 3://bring tote over second tote
			//tank.TankDrive(FWD_SPEED, FWD_SPEED);
			drive_balance = 1 + (leftCode.GetDistance() + -rightCode.GetDistance()) * FB_GAIN;
			tank.TankDrive(FWD_SPEED / drive_balance, FWD_SPEED * drive_balance);
			if (leftCode.GetDistance() >= 10){
				toteState = 4;
			}
			break;
		case 4://stop and drop
			//tank.TankDrive(FWD_HOLD_SPEED, FWD_HOLD_SPEED);
			drive_balance = 1 + (leftCode.GetDistance() + -rightCode.GetDistance()) *  FB_GAIN;
			tank.TankDrive(FWD_HOLD_SPEED / drive_balance, FWD_HOLD_SPEED * drive_balance);
			pickupInch = 0;
			if (current_position <= 0){
				toteState = 5;
				last_distance = 0;
				leftCode.Reset();
				rightCode.Reset();
			}
			break;
		case 5://drive to second tote and pick up
			//tank.TankDrive(FWD_SPEED, FWD_SPEED);
			drive_balance = 1 + (leftCode.GetDistance() + -rightCode.GetDistance()) * FB_GAIN;
			tank.TankDrive(FWD_SPEED / drive_balance, FWD_SPEED * drive_balance);
			if ((leftCode.GetDistance() > 1) && (leftCode.GetDistance() - last_distance < 0.01)){
				max_speed = 5;
				pickupInch = 20;
				toteState = 6;
			}
			last_distance = leftCode.GetDistance();
			break;
		case 6://stop
			tank.TankDrive(0.0, 0.0);
			if (current_position >= 2){
				max_speed = 10;
				claw.Set(true);
			}
			if (current_position >= 5){
				leftCode.Reset();
				rightCode.Reset();
				toteState = 7;
			}
			break;
		case 7:
			drive_balance = 1 + (-leftCode.GetDistance() + rightCode.GetDistance()) * FB_GAIN;
			tank.TankDrive(-FWD_SPEED / drive_balance, -FWD_SPEED * drive_balance);
			if (leftCode.GetDistance() <= -21){
				toteState = 8;
				autoTime = 0;
			}
			break;
		case 8:
			tank.TankDrive(0.0, 0.0);
			Cylinders.Set(true);
			if (time > 25){
				fStrafe.Set(-landfillMove.Position(autoTime) * magic * FRONT_STRAFE_FF);
				bStrafe.Set(landfillMove.Position(autoTime) * magic * REAR_STRAFE_FF);
				autoTime += delta_time;
			}
			time++;
			if (getFrontStrafePosition() <= -60.4){
				time = 0;
				toteState = 9;
				//fStrafe.SetPosition(0);
				//bStrafe.SetPosition(0);
			}
			break;
		case 9:
			tank.TankDrive(0.0, 0.0);
			Cylinders.Set(false);
			fStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			bStrafe.SetControlMode(CANSpeedController::kPercentVbus);
			fStrafe.Set(0.0);
			bStrafe.Set(0.0);
			leftCode.Reset();
			rightCode.Reset();
			last_distance = 0;
			claw.Set(false);
			pickupInch = 0;
			toteState = 10;
			break;
		case 10:
			tank.TankDrive(0.0, 0.0);
			if (current_position <= 0){
				leftCode.Reset();
				rightCode.Reset();
				last_distance = 0;
				toteState = 11;
			}
			break;
		case 11:
			//tank.TankDrive(FWD_SPEED + 0.15, FWD_SPEED + 0.15);
			drive_balance = 1 + (leftCode.GetDistance() + -rightCode.GetDistance()) * FB_GAIN;
			tank.TankDrive(FWD_SPEED / drive_balance, FWD_SPEED * drive_balance);
			if ((leftCode.GetDistance() > 1) && (leftCode.GetDistance() - last_distance < 0.005)){
				pickupInch = 17;
				toteState = 12;
			}
			last_distance = leftCode.GetDistance();
			break;
		case 12:
			tank.TankDrive(0.0, 0.0);
			if (current_position >= 2){
				claw.Set(true);
			}
			if (current_position >= 16){
				leftCode.Reset();
				rightCode.Reset();
				toteState = 13;
			}
			break;
		case 13:
			drive_balance = 1 + (leftCode.GetDistance() + -rightCode.GetDistance()) * FB_GAIN;
			tank.TankDrive(FWD_SPEED / drive_balance, FWD_SPEED * drive_balance);
			//tank.TankDrive(FWD_SPEED, FWD_SPEED);
			if (leftCode.GetDistance() >= 10){
				toteState = 14;
			}
			break;
		case 14:
			//tank.TankDrive(FWD_HOLD_SPEED, FWD_HOLD_SPEED);
			drive_balance = 1 + (leftCode.GetDistance() + -rightCode.GetDistance()) * FB_GAIN;
			tank.TankDrive(FWD_HOLD_SPEED / drive_balance, FWD_HOLD_SPEED * drive_balance);
				pickupInch = 12;
				if (current_position <= 13){
					leftCode.Reset();
					rightCode.Reset();
					toteState = 15;
				}
			break;
		case 15:
			tank.TankDrive(0.4, 0.4);
			if (leftCode.GetDistance() >= 1){
				toteState = 16;
			}
			break;
		case 16:
			tank.TankDrive(0.0, 0.0);
			break;
	}

}
};

START_ROBOT_CLASS(Robot);
