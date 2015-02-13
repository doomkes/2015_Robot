#include "WPILib.h"
#include "RobotMap.h"
#include "Autonomous.h"

class Robot: public IterativeRobot
{
	RobotDrive tank;	//normal drive wheels tank drive
	CANTalon lift;
	Victor fStrafe, bStrafe;	//lift and 2 strafe motors
	Victor  fStrafe2, bStrafe2;
	Joystick lStick, rStick, liftStick;
	Solenoid Cylinders;	//solenoids that control strafing wheel height
	Ultrasonic ultra;
	SendableChooser autonChooser;
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

//private:

	//Command *autonomousCommand;
	//SendableChooser *chooser;

public:

	Robot():
		tank(LEFT_MOTOR, 4, RIGHT_MOTOR, 6),
		lift(0),
		fStrafe(FRONT_STRAFE_MOTOR),
		bStrafe(BACK_STRAFE_MOTOR_1),
		fStrafe2(5),
		bStrafe2(7),
		lStick(LTANK_JOY_USB),
		rStick(RTANK_JOY_USB),
		liftStick(LIFT_JOY_USB),
		Cylinders(CYLINDERS),
		ultra(8, 9)


	{
		tank.SetExpiration(0.1);
	}

void RobotInit()
{
	CameraServer::GetInstance()->SetQuality(50);
	CameraServer::GetInstance()->StartAutomaticCapture("cam2");

	lift.SetFeedbackDevice(CANTalon::QuadEncoder);
	lift.SelectProfileSlot(1);
	lift.SetPID(4, 0.01, 0);
	lift.SetIzone(512);
	lift.SetCloseLoopRampRate(300);
	lift.SetPosition(0);

	SmartDashboard::PutNumber("autoValue", 1);
	SmartDashboard::PutString("autoChoose", "Yellow Tote = 1          Robot Set = 2         Container = 3");
	//chooser = new SendableChooser();
	//autonChooser.AddDefault("auto 1", yellowTote());
	//autonChooser.AddObject("auto 2", new pickContainer());
	//SmartDashboard::PutData("Autonomous modes", autonChooser);

}

void AutonomousInit()
{
	delta_time = 0;
	//autonomousCommand = (Command *) chooser->GetSelected();
	//autonomousCommand->Start();
	switchInt = 1;
	time = 0;
	lift.SetPosition(0);
	lift.SetControlMode(CANSpeedController::kPosition);

	autoProgram = SmartDashboard::GetNumber("autoValue", 1.000);
}

void AutonomousPeriodic()
{
	//Scheduler::GetInstance()->Run();
	if (autoProgram == 1) YellowTote();
	if (autoProgram == 2) RobotSet();
	if (autoProgram == 3) Container();
	encoderControl();
	printf("%f \n", delta_time);
}
void TeleopInit()
{

}

void TeleopPeriodic()
{
	ultra.SetAutomaticMode(true);
	//int range = ultra.GetRangeInches();
	//printf("range = %d\n", range);

	encoderControl();
	leftJoyX = lStick.GetX();
	leftJoyY = lStick.GetY();
	rightJoyX = rStick.GetX();
	rightJoyY = rStick.GetY();
	liftJoy = liftStick.GetRawAxis(4);
	//printf("lift joystick value = %f", liftJoy);

	if (lStick.GetRawButton(3)) {	//turbo mode
		if (((leftJoyY-rightJoyY) <= 0.1)&&((leftJoyY-rightJoyY) >= -0.1)){		//10% range
			leftJoyY = (leftJoyY+rightJoyY)/2;
			rightJoyY = (leftJoyY+rightJoyY)/2;
		}
		turbo_mode = 1.0;
		}
	else turbo_mode = 0.7;

	printf("%f %f\n",leftJoyY, rightJoyY);

	if ((lStick.GetRawButton(1)) || (rStick.GetRawButton(1))) {	//strafing
		frontVal = leftJoyX;
		rearVal = -rightJoyX;
		Cylinders.Set(true);
		tank.TankDrive(0.0 ,0.0 ,false);
	}
	else if ((lStick.GetRawButton(2)) || (rStick.GetRawButton(2)))	{	//strait strafe
		Cylinders.Set(true);
		tank.TankDrive(0.0 ,0.0 ,false);
		frontVal = (leftJoyX + rightJoyX) / 2;
		rearVal = (leftJoyX + rightJoyX) / -3;
	}
	else {			//non strafing
		Cylinders.Set(false);
		tank.TankDrive(-leftJoyY * turbo_mode, -rightJoyY * turbo_mode, true);
		frontVal = 0.0;
		rearVal = 0.0;
	}
	//motors are always "running", but only actually do anything when the values don't equal 0
	fStrafe.Set(frontVal);
	bStrafe.Set(rearVal);

	if (liftStick.GetRawButton(1))	//stage 1
		{
		pickupInch = 7;
		triggeralreadyPressed = false;
		}
	if (liftStick.GetRawButton(2))	//stage 2
		{
		pickupInch = 19.1;
		triggeralreadyPressed = false;
		}
	if (liftStick.GetRawButton(3))	//stage 3
		{
		pickupInch = 31.2;
		triggeralreadyPressed = false;
		}
	if (liftStick.GetRawButton(4))	//stage 4
		{
		pickupInch = 43.3;
		triggeralreadyPressed = false;
		}
	if (liftStick.GetRawButton(6))	//stage 5
		{
		pickupInch = 55.4;
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
	if (liftStick.GetRawButton(9))	//zero encoder
		{
		lift.SetPosition(0);
		pickupInch = 0;
		}
	if (liftStick.GetRawButton(5))	//manual control on
	{
		manualControl = true;
	}
	if (liftStick.GetRawButton(7))	//manual control off
	{
		manualControl = false;
	}

	if (manualControl)
	{
		lift.SetControlMode(CANSpeedController::kPercentVbus);
		lift.Set(-liftJoy);
		delta_time = 0;
	}
	else lift.SetControlMode(CANSpeedController::kPosition);

}

//
void TestPeriodic()
{

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
	lift.Set(-current_position * 91.428);
	last_time = Timer::GetFPGATimestamp();
	//printf("delta time %f     current position %f      pickupInch %f      encoder Value %i \n", delta_time, current_position, pickupInch, lift.GetEncPosition());
}

void Container()	//auto
{
	switch (switchInt)
		{
			case 1:	//bring lift up to pick up container
				printf("case 1\n");
				pickupInch = 24;
				switchInt = 2;
				break;
			case 2:	//wait for lift to get to position
				printf("case 2\n");
				if (time >= 10)
					{
					switchInt = 3;
					time = 0;
					}
				else time = time  + delta_time;
				break;
			case 3:	//drive into auto zone
			printf("case 3\n");
				tank.TankDrive(-0.2, -0.2, false);
				if (time >= 0.0001) switchInt = 4;
				else time = time + delta_time;
				break;
			case 4:	//stop in auto zone
			printf("case 4\n");
				tank.TankDrive(0.0, 0.0, false);
				switchInt = 5;
				time = 0;
				break;
			case 5:	//bring lift down
			printf("case 5\n");
				pickupInch = 0;
				if (time >= 3.0)
					{
					switchInt = 6;
					time = 0;
					}
				else time = time + delta_time;
				break;
			case 6:	//back away
			printf("case 6\n");
				tank.TankDrive(-0.5, -0.5, false);
				if (time >= 0.2) switchInt = 7;
				else time = time + delta_time;
				break;
			case 7:	//stop
			printf("case 7\n");
				tank.TankDrive(0.0, 0.0, false);
				break;
			default:
				printf("%i \n", switchInt);
		}
}
void RobotSet()		//auto
{
	switch (switchInt)			//robot set
		{
			case 1:	//drive into auto zone
				tank.TankDrive(-0.5, -0.5, false);
				if (time >= 800) switchInt = 2;
				else time = time + 20;
				break;
			case 2:	//stop
				tank.TankDrive(0.0, 0.0, false);
				break;

		}
}
void YellowTote()	//auto
{

	switch (switchInt)
	{
		case 1:	//pick up tote
			printf("case 1\n");
			pickupInch = 8;
			switchInt = 2;
			break;
		case 2:	//wait for tote to rise
			printf("case 2\n");
			if (time >= 500)
				{
				switchInt = 3;
				time = 0;
				}
			else time = time  + 20;
			break;
		case 3:	//tote will become your master
		printf("case 3\n");
			tank.TankDrive(-0.3, -0.3, false);
			if (time >= 2450) {
				switchInt = 4;
			}
			else time = time + 20;
			break;
		case 4:	//be slave to tote
		printf("case 4\n");
			tank.TankDrive(0.0, 0.0, false);
			Cylinders.Set(true);
			switchInt = 5;
			time = 0;
			break;
		case 5:	//you will totes regret ever making a robot
		printf("case 5\n");
			pickupInch = 0;
			if (time >= 1600)
				{
				Cylinders.Set(false);
				switchInt = 6;
				time = 0;
				}
			else time = time + 20;
			break;
		case 6:	//"die you foolish human" ~Tote
		printf("case 6\n");
			tank.TankDrive(-0.2, -0.2, false);
			if (time >= 100) switchInt = 7;
			else time = time + 20;
			break;
		case 7:	//soon, the world will be lost...
		printf("case 7\n");
			tank.TankDrive(0.0, 0.0, false);
			break;
		default:
			printf("%i \n", switchInt);
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
