#include "Autonomous.h"

void drive(frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, double distance, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	DriverStation& ds = DriverStation::GetInstance();
	leftController.SetSetpoint(distance);
	rightController.SetSetpoint(-distance);
	lMotors.SetInverted(true);
	rMotors.SetInverted(true);

	leftController.Enable();
	rightController.Enable();
	while(!(leftController.OnTarget() && rightController.OnTarget()) && ds.IsAutonomous())
	{
		if(navx->GetAngle()>2.5)
		{
			rightController.Enable();
			lMotors.Set(leftController.Get()*.7);
		}
		else if(navx->GetAngle()<-2.5)
		{
			leftController.Enable();
			rMotors.Set(rightController.Get()*.7);
		}
		else
		{
			leftController.Enable();
			rightController.Enable();
		}
		frc::Wait(0.005);
		std::cout << "Left" << leftEnc.GetDistance() << ", Right" << rightEnc.GetDistance() << "Angle" << navx->GetAngle() <<std::endl;
	}
	leftController.Disable();
	rightController.Disable();
	rMotors.Set(0);
	lMotors.Set(0);
}

void liftSwitch(frc::SpeedControllerGroup& lift)
{
	lift.Set(.5);
	frc::Wait(2);
	lift.Set(0);
}

void liftScale(frc::SpeedControllerGroup& lift)
{
	lift.Set(.5);
	frc::Wait(4);
	lift.Set(0);
}

void turn(frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, double angle, AHRS* navx)
{
	DriverStation& ds = DriverStation::GetInstance();
	if(angle>0)
	{
		while(navx->GetAngle()<angle  && ds.IsAutonomous())
		{
			lMotors.Set(.65);
			rMotors.Set(.65);
			frc::Wait(0.001);
		}
	}
	else
	{
		while(navx->GetAngle()>angle  && ds.IsAutonomous())
		{
			lMotors.Set(-.65);
			rMotors.Set(-.65);
			frc::Wait(0.001);
		}
	}
	lMotors.Set(0);
	rMotors.Set(0);
	navx->Reset();
	std::cout << "Angle" << navx->GetAngle() << std::endl;
}

void drop(frc::SpeedControllerGroup& intake)
{
	intake.Set(.5);
	frc::Wait(.5);
	intake.Set(0);
}

void encoderRes(frc::Encoder& leftEnc, frc::Encoder& rightEnc)
{
	leftEnc.Reset();
	rightEnc.Reset();
}

void mobility(frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	drive(leftController, rightController, lMotors, rMotors, 120, leftEnc, rightEnc, navx);
}

void leftSwitch(frc::DifferentialDrive& Robotdrive, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	drive(leftController, rightController, lMotors, rMotors, 168, leftEnc, rightEnc, navx);
	turn(lMotors, rMotors, 45, navx);
	encoderRes(leftEnc, rightEnc);
	liftSwitch(lift);
	drive(leftController, rightController, lMotors, rMotors, 18, leftEnc, rightEnc, navx);
	drop(intake);
}
void rightSwitch(frc::DifferentialDrive& Robotdrive, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	drive(leftController, rightController, lMotors, rMotors, 168, leftEnc, rightEnc, navx);
	turn(lMotors, rMotors, -45, navx);
	encoderRes(leftEnc, rightEnc);
	liftSwitch(lift);
	drive(leftController, rightController, lMotors, rMotors, 18, leftEnc, rightEnc, navx);
	drop(intake);
}
void leftScale(frc::DifferentialDrive& Robotdrive, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	drive(leftController, rightController, lMotors, rMotors, 288, leftEnc, rightEnc, navx);
	turn(lMotors, rMotors, 30, navx);
	encoderRes(leftEnc, rightEnc);
	liftScale(lift);
	drive(leftController, rightController, lMotors, rMotors, 24, leftEnc, rightEnc, navx);
	drop(intake);
}
void rightScale(frc::DifferentialDrive& Robotdrive, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	drive(leftController, rightController, lMotors, rMotors, 288, leftEnc, rightEnc, navx);
	turn(lMotors, rMotors, -30, navx);
	encoderRes(leftEnc, rightEnc);
	liftScale(lift);
	drive(leftController, rightController, lMotors, rMotors, 24, leftEnc, rightEnc, navx);
	drop(intake);
}
void leftMid(frc::DifferentialDrive& Robotdrive, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	turn(lMotors, rMotors, -45, navx);
	encoderRes(leftEnc, rightEnc);
	drive(leftController, rightController, lMotors, rMotors, 76, leftEnc, rightEnc, navx);
	turn(lMotors, rMotors, 45, navx);
	encoderRes(leftEnc, rightEnc);
	liftSwitch(lift);
	drive(leftController, rightController, lMotors, rMotors, 72, leftEnc, rightEnc, navx);
	drop(intake);
}
void rightMid(frc::DifferentialDrive& Robotdrive, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx)
{
	turn(lMotors, rMotors, 45, navx);
	encoderRes(leftEnc, rightEnc);
	drive(leftController, rightController, lMotors, rMotors, 76, leftEnc, rightEnc, navx);
	turn(lMotors, rMotors, -45, navx);
	encoderRes(leftEnc, rightEnc);
	liftSwitch(lift);
	drive(leftController, rightController, lMotors, rMotors, 72, leftEnc, rightEnc, navx);
	drop(intake);
}



