/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <XboxController.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Talon.h>
#include <PWMTalonSRX.h>
#include <DoubleSolenoid.h>
#include <Timer.h>
#include <AHRS.h>
#include <Encoder.h>
#include <DigitalInput.h>

#include "Autonomous.h"

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * The SampleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at the right
 * time as controlled by the switches on the driver station or the field
 * controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
class Robot : public frc::SampleRobot {
public:
	Robot() {
		robotDrive.SetExpiration(0.1);
		navx = new AHRS(SPI::Port::kMXP);
		try
		{
		   navx = new AHRS(SPI::Port::kMXP);
		   } catch (std::exception ex ) {
		   std::string err_string = "Error instantiating navX-MXP:  ";
		   err_string += ex.what();
		   DriverStation::ReportError(err_string.c_str());
		}
		intakeLeft.SetInverted(true);
		shifts = 0;
	}

	void RobotInit() {
		//m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		//m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		//frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		compressor.Start();

		leftEnc.SetDistancePerPulse(encoderScale);
		rightEnc.SetDistancePerPulse(encoderScale);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select between different autonomous modes using the dashboard. The
	 * sendable chooser code works with the Java SmartDashboard. If you
	 * prefer the LabVIEW Dashboard, remove all of the chooser code and
	 * uncomment the GetString line to get the auto name from the text box
	 * below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void Autonomous() {
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		leftEnc.Reset();
		rightEnc.Reset();
		navx->Reset();

		frc::PIDController lDriveController{kP, kI, kD, &leftEnc, &lDriveMotors};
		frc::PIDController rDriveController{kP, kI, kD, &rightEnc, &rDriveMotors};


		lDriveController.Disable();
		rDriveController.Disable();

		lDriveController.SetOutputRange(-.4, .4);
		rDriveController.SetOutputRange(-.4, .4);

		lDriveController.SetAbsoluteTolerance(.5);
		rDriveController.SetAbsoluteTolerance(.5);

		std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameMobility);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		robotDrive.SetSafetyEnabled(false);

		if (autoSelected == kAutoNameMobility)
		{
			std::cout << "Beginning Mobility" << std::endl;
			mobility(lDriveController, rDriveController, lDriveMotors, rDriveMotors, leftEnc, rightEnc, navx);
		}
		else if (autoSelected == kAutoNameSwitchL)
		{
			if(gameData[0] == 'L')
			{
				std::cout << "Beginning Left Switch" << std::endl;
				leftSwitch(robotDrive, kt, lDriveController, rDriveController, lDriveMotors, rDriveMotors, lift, intake, leftEnc, rightEnc, navx);
			}
			else
			{
				std::cout << "Sorry Left Switch Failed; Performing Mobility" << std::endl;
				mobility(lDriveController, rDriveController, lDriveMotors, rDriveMotors, leftEnc, rightEnc, navx);
			}
		}
		else if (autoSelected == kAutoNameSwitchR)
		{
			if(gameData[0] == 'R')
			{
				std::cout << "Beginning Right Switch" << std::endl;
				rightSwitch(robotDrive, kt, lDriveController, rDriveController, lDriveMotors, rDriveMotors, lift, intake, leftEnc, rightEnc, navx);
			}
			else
			{
				std::cout << "Sorry Right Switch Failed; Performing Mobility" << std::endl;
				mobility(lDriveController, rDriveController, lDriveMotors, rDriveMotors, leftEnc, rightEnc, navx);
			}
		}
		else if (autoSelected == kAutoNameScaleL)
		{
			if(gameData[1] == 'L')
			{
				std::cout << "Beginning Left Scale" << std::endl;
				leftScale(robotDrive, kt, lDriveController, rDriveController, lDriveMotors, rDriveMotors, lift, intake, leftEnc, rightEnc, navx);
			}
			else
			{
				std::cout << "Sorry Left Scale Failed; Performing Mobility" << std::endl;
				mobility(lDriveController, rDriveController, lDriveMotors, rDriveMotors, leftEnc, rightEnc, navx);
			}
		}
		else if (autoSelected == kAutoNameScaleR)
		{
			if(gameData[1] == 'R')
			{
				std::cout << "Beginning Right Scale" << std::endl;
				rightScale(robotDrive, kt, lDriveController, rDriveController, lDriveMotors, rDriveMotors, lift, intake, leftEnc, rightEnc, navx);
			}
			else
			{
				std::cout << "Sorry Right Scale Failed; Performing Mobility" << std::endl;
				mobility(lDriveController, rDriveController, lDriveMotors, rDriveMotors, leftEnc, rightEnc, navx);
			}
		}
	}

	/*
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() override {
		robotDrive.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled())
			{
			std::cout << "Left" << leftEnc.GetDistance() << ", Right" << rightEnc.GetDistance() << "Angle" << navx->GetAngle() <<std::endl;
			// Drive
			robotDrive.TankDrive(controller.GetY(frc::GenericHID::kLeftHand), controller.GetY(frc::GenericHID::kRightHand));
			// Lift
			lift.Set(controller.GetTriggerAxis(frc::GenericHID::kLeftHand)-controller.GetTriggerAxis(frc::GenericHID::kRightHand));

			if (!Hall.Get())
			{
				lift.Set((controller.GetTriggerAxis(frc::GenericHID::kLeftHand)-controller.GetTriggerAxis(frc::GenericHID::kRightHand))*kH);
			}
			else
			{
				lift.Set(controller.GetTriggerAxis(frc::GenericHID::kLeftHand)-controller.GetTriggerAxis(frc::GenericHID::kRightHand));
			}

			// Intake
			if (controller.GetBumper(frc::GenericHID::kLeftHand))
			{
				intake.Set( .7);
			}
			else if (controller.GetBumper(frc::GenericHID::kRightHand))
			{
				intake.Set(-.7);
			}
			else
			{
				intake.Set(0);
			}
			if (controller.GetStickButton(frc::GenericHID::kLeftHand))
			{
				shifter.Set(frc::DoubleSolenoid::kForward);
			}
			else if (controller.GetStickButton(frc::GenericHID::kRightHand))
			{
				shifter.Set(frc::DoubleSolenoid::kReverse);
			}
			else
			{
				shifter.Set(frc::DoubleSolenoid::kOff);
			}
			// The motors will be updated every 5ms
			frc::Wait(0.005);
		}
	}

	/*
	 * Runs during test mode
	 */
	void Test() override {}

private:
	// ROBOT DRIVE SYSTEM
	frc::PWMTalonSRX f_leftMotor{0};
	frc::PWMTalonSRX b_leftMotor{1};
	frc::PWMTalonSRX f_rightMotor{2};
	frc::PWMTalonSRX b_rightMotor{3};

	frc::SpeedControllerGroup lDriveMotors{f_leftMotor, b_leftMotor};
	frc::SpeedControllerGroup rDriveMotors{f_rightMotor, b_rightMotor};

	frc::DifferentialDrive robotDrive{lDriveMotors, rDriveMotors};
	// ROBOT LIFT
	frc::PWMTalonSRX liftOne{4};
	frc::PWMTalonSRX liftTwo{5};

	frc::SpeedControllerGroup lift{liftOne, liftTwo};
	// ROBOT INTAKE
	frc::Talon intakeLeft{6};
	frc::Talon intakeRight{7};

	frc::SpeedControllerGroup intake{intakeLeft, intakeRight};

	frc::XboxController controller{0};
	// PNUEMATIC SHIFTING
	frc::DoubleSolenoid shifter{0, 1};
	// NAVX GYRO
	AHRS *navx;
	// ENCODERS
	frc::Encoder leftEnc{4, 5, false, frc::CounterBase::EncodingType::k4X};
	frc::Encoder rightEnc{2, 3, false, frc::CounterBase::EncodingType::k4X};
	const double encoderScale = ((3.141592 * 6) / 2048);
	int shifts;
	// HALL EFFECTS
	frc::DigitalInput Hall{1};
	const double kH = .2;
	// Compressor
	frc::Compressor compressor;
	//PID Controllers

	//frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameMobility = "Mobility";
	const std::string kAutoNameSwitchL = "L Switch";
	const std::string kAutoNameSwitchR = "R Switch";
	const std::string kAutoNameScaleL = "L Scale";
	const std::string kAutoNameScaleR = "R Scale";

	const double kP = .35;
	const double kI = 0;
	const double kD = 0;
	const double kt = .01;
};

START_ROBOT_CLASS(Robot)
