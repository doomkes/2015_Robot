#include "WPILib.h"

class Robot: public IterativeRobot
{
	RobotDrive tank; //normal drive wheels tank drive
	Joystick lStick, rStick, liftStick;
	Victor lift, fStrafe1, fStrafe2, bStrafe1, bStrafe2;
	int frontVal = 0;
	int rearVal = 0;
	double leftJoy = 0;
	double rightJoy = 0;
	Solenoid front, back;

public:
	Robot() :
			tank(0, 1, 8, 9),
			lStick(0),
			rStick(1),
			liftStick(2),
			lift(6),
			fStrafe1(2),
			fStrafe2(3),
			bStrafe1(6),
			bStrafe2(5),
			front(1),
			back(2)
	{
		tank.SetExpiration(0.1);
	}

void RobotInit()
{

}

void AutonomousInit()
{

}

void AutonomousPeriodic()
{

}

void TeleopInit()
{

}

void TeleopPeriodic()
{
	leftJoy = lStick.GetY();
	rightJoy = rStick.GetY();
	lift.Set(liftStick.GetY());

	if(lStick.GetRawButton(3))
	{
		frontVal = 1;
		rearVal = -1;
		front.Set(true);
		back.Set(true);
	}
	else if (rStick.GetRawButton(3))
	{
		frontVal = -1;
		rearVal = 1;
		front.Set(true);
		back.Set(true);
	}
	else
	{
		frontVal = 0;
		rearVal = 0;
		front.Set(false);
		back.Set(false);
		tank.TankDrive(-leftJoy, -rightJoy, true);

	}

	fStrafe1.Set(frontVal);
	fStrafe2.Set(frontVal);
	bStrafe1.Set(rearVal);
	bStrafe2.Set(frontVal);

}

void TestPeriodic()
{

}
};

START_ROBOT_CLASS(Robot);
