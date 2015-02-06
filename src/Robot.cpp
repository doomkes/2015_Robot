#include "WPILib.h"
#include "RobotMap.h"
#include "Autonomous.h"

class Robot: public IterativeRobot
{
	RobotDrive tank;	//normal drive wheels tank drive
	CANTalon lift;
	Victor fStrafe, bStrafe;	//lift and 2 strafe motors
	Joystick lStick, rStick, liftStick;
	Solenoid Cylinders;	//solenoids that control strafing wheel height
	float frontVal = 0;
	float rearVal = 0;
	float leftJoyX = 0;
	float leftJoyY = 0;
	float rightJoyX = 0;
	float rightJoyY = 0;
	float liftJoy = 0;
	float encoderVal = 11.2;


//private:

	//Command *autonomousCommand;
	//SendableChooser *chooser;

public:

	Robot():
		tank(LEFT_MOTOR, RIGHT_MOTOR),
		lift(0),
		fStrafe(FRONT_STRAFE_MOTOR),
		bStrafe(BACK_STRAFE_MOTOR_1),
		lStick(LTANK_JOY_USB),
		rStick(RTANK_JOY_USB),
		liftStick(LIFT_JOY_USB),
		Cylinders(CYLINDERS)


	{
		tank.SetExpiration(0.1);
	}

void RobotInit()
{

	//chooser = new SendableChooser();
	//chooser->AddDefault("auto 1", new yellowTote());
	//chooser->AddObject("auto 2", new pickContainer());
	//SmartDashboard::PutData("Autonomous modes", chooser);

}

void AutonomousInit()
{
	//autonomousCommand = (Command *) chooser->GetSelected();
	//autonomousCommand->Start();
}

void AutonomousPeriodic()
{
	//Scheduler::GetInstance()->Run();
}

void TeleopInit()
{
	lift.SetPosition(0);
	SmartDashboard::PutNumber("auto", 1);
	SmartDashboard::PutString("Auto 1","1 Yellow Tote");
	SmartDashboard::PutString("Auto 2","3 Yellow Totes");
}

void TeleopPeriodic()
{

	encoderControl();
	leftJoyX = lStick.GetX();
	leftJoyY = lStick.GetY();
	rightJoyX = rStick.GetX();
	rightJoyY = rStick.GetY();
	liftJoy = liftStick.GetY();

	if ((lStick.GetRawButton(1)) || (rStick.GetRawButton(1))) {
		frontVal = leftJoyX;
		rearVal = -rightJoyX;
		Cylinders.Set(true);
		tank.TankDrive(0.0 ,0.0 ,false);
	}
	else {
		Cylinders.Set(false);
		tank.TankDrive(-leftJoyY, -rightJoyY, false);
		frontVal = 0.0;
		rearVal = 0.0;
	}

	//motors are always "running", but only actually do anything when the values don't equal 0
	fStrafe.Set(frontVal);
	bStrafe.Set(rearVal);
	lift.Set(-liftJoy);





}

void TestPeriodic()
{

}

void encoderControl()
{
	/*lift.SetControlMode(CANSpeedController::kPosition);
	lift.SetFeedbackDevice(CANTalon::QuadEncoder);
	lift.SelectProfileSlot(1);
	lift.SetPID(4, 0.01, 0);
	lift.SetIzone(512);
	lift.SetCloseLoopRampRate(300);
	lift.Set(encoderVal * 91.428);
	lift.SetSensorDirection(true);
	printf("%i \n",lift.GetEncPosition());
	*/
}
void yellowTote()
{

}
void pickContainer()
{

}
};

START_ROBOT_CLASS(Robot);
