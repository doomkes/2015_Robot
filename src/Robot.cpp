#include "WPILib.h"
#include "RobotMap.h"

class Robot: public IterativeRobot
{
	RobotDrive tank;	//normal drive wheels tank drive
	Victor lift, fStrafe1, fStrafe2, bStrafe1, bStrafe2;	//lift and 4 strafe motors
	Joystick lStick, rStick, liftStick;
	Solenoid fCylinder, bCylinder;	//solenoids that control strafing wheel height
	float frontVal = 0;
	float rearVal = 0;
	float leftJoy = 0;
	float rightJoy = 0;
	float liftJoy = 0;

public:
	Robot():
		tank(LEFT_MOTOR_1, LEFT_MOTOR_2, RIGHT_MOTOR_1, RIGHT_MOTOR_2),
		lift(LIFT_MOTOR),
		fStrafe1(FRONT_STRAFE_MOTOR_1),
		fStrafe2(FRONT_STRAFE_MOTOR_2),
		bStrafe1(BACK_STRAFE_MOTOR_1),
		bStrafe2(BACK_STRAFE_MOTOR_2),
		lStick(LTANK_JOY_USB),
		rStick(RTANK_JOY_USB),
		liftStick(LIFT_JOY_USB),
		fCylinder(FRONT_CYLINDER),
		bCylinder(BACK_CYLINDER)
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
	liftJoy = liftStick.GetY();

	//when strafing left
	if(lStick.GetRawButton(3))
	{
		frontVal = 1;
		rearVal = -1;
		fCylinder.Set(true);
		bCylinder.Set(true);
	}
	//strafing right
	else if (rStick.GetRawButton(3))
	{
		frontVal = -1;
		rearVal = 1;
		fCylinder.Set(true);
		bCylinder.Set(true);
	}
	//no strafing, normal tank drive
	else
	{
		frontVal = 0;
		rearVal = 0;
		fCylinder.Set(false);
		bCylinder.Set(false);
		tank.TankDrive(-leftJoy, -rightJoy, true);

	}

	//motors are always "running", but only actually do anything when the values don't equal 0
	fStrafe1.Set(frontVal);
	fStrafe2.Set(frontVal);
	bStrafe1.Set(rearVal);
	bStrafe2.Set(rearVal);

	lift.Set(-liftJoy);
}

void TestPeriodic()
{

}
};

START_ROBOT_CLASS(Robot);
