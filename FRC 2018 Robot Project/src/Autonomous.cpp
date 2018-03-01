#include "Autonomous.h"

void drive(frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, double distance, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	leftController.SetSetpoint(-distance);
	rightController.SetSetpoint(distance);

	leftController.Enable();
	rightController.Enable();
	while(!(leftController.OnTarget() && rightController.OnTarget()))
	{
		if(navx->GetAngle()>10)
		{
			leftController.Enable();
			rMotors.Set(rightController.Get()*.8);
		}
		else if(navx->GetAngle()<-10)
		{
			rightController.Enable();
			lMotors.Set(leftController.Get()*.8);
		}
		else
		{
			leftController.Enable();
			rightController.Enable();
		}
		frc::Wait(0.001);
		std::cout << "Left" << leftEnc.GetDistance() << ", Right" << rightEnc.GetDistance() << std::endl;
	}
	leftController.Disable();
	rightController.Disable();
	rMotors.Set(0);
	lMotors.Set(0);
}

void liftSwitch(frc::SpeedControllerGroup& lift)
{
	lift.Set(.7);
	frc::Wait(.75);
	lift.Set(0);
}

void liftScale(frc::SpeedControllerGroup& lift)
{
	lift.Set(.7);
	frc::Wait(1.5);
	lift.Set(0);
}

void turn(frc::DifferentialDrive& drive, double angle, double kt, AHRS* navx)
{
	if(angle>0)
	{
		while((navx->GetAngle()+5)<angle)
		{
			drive.CurvatureDrive(.25, ((angle-navx->GetAngle())*kt), true);
			frc::Wait(0.001);
		}
	}
	else if(angle<0)
	{
		while((navx->GetAngle()-5)>angle)
		{
			drive.CurvatureDrive(.25, ((angle-navx->GetAngle())*kt), true);
			frc::Wait(0.001);
		}
	}
}

void drop(frc::SpeedControllerGroup& intake)
{
	intake.Set(-.5);
	frc::Wait(.5);
	intake.Set(0);
}

void mobility(frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	drive(leftController, rightController, lMotors, rMotors, 120, leftEnc, rightEnc, navx);
}

void leftSwitch(frc::DifferentialDrive& Robotdrive, double kt, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	liftSwitch(lift);
	drive(leftController, rightController, lMotors, rMotors, 168, leftEnc, rightEnc, navx);
	turn(Robotdrive, -90, kt, navx);
	drop(intake);
}

void rightSwitch(frc::DifferentialDrive& Robotdrive, double kt, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	liftSwitch(lift);
	drive(leftController, rightController, lMotors, rMotors, 168, leftEnc, rightEnc, navx);
	turn(Robotdrive, 90, kt, navx);
	drop(intake);
}

void leftScale(frc::DifferentialDrive& Robotdrive, double kt, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	liftScale(lift);
	drive(leftController, rightController, lMotors, rMotors, 240, leftEnc, rightEnc, navx);
	turn(Robotdrive, -30, kt, navx);
	drop(intake);
}

void rightScale(frc::DifferentialDrive& Robotdrive, double kt, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	liftScale(lift);
	drive(leftController, rightController, lMotors, rMotors, 240, leftEnc, rightEnc, navx);
	turn(Robotdrive, 30, kt, navx);
	drop(intake);
}

