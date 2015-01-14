#include "WPILib.h"

class Robot: public IterativeRobot
{
	RobotDrive tank; //normal drive wheels tank drive
	Joystick lStick, rStick;
	Talon fMiddle, bMiddle; //strafe motors
	int frontVal = 0;
	int rearVal = 0;
	double leftJoy = 0;
	double rightJoy = 0;

public:
	Robot() :
			tank(0, 1, 8, 9),
			lStick(0),
			rStick(1),
			fMiddle(6),
			bMiddle(7)
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
	tank.TankDrive(-leftJoy, -rightJoy, true);

	if(lStick.GetRawButton(3))
	{
		frontVal = 1;
		rearVal = -1;
	}
	else if (rStick.GetRawButton(3))
	{
			frontVal = -1;
			rearVal = 1;
	}
	else
		{
			frontVal = 0;
			rearVal = 0;
		}

	fMiddle.Set(frontVal);
	bMiddle.Set(rearVal);
}

void TestPeriodic()
{

}
};

START_ROBOT_CLASS(Robot);
