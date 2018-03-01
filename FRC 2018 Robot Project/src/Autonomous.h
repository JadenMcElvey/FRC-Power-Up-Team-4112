#ifndef AUTO_H
#define AUTO_H

#include <iostream>

#include <Drive/DifferentialDrive.h>
#include <PWMTalonSRX.h>
#include <AHRS.h>
#include <Encoder.h>
#include <Timer.h>

void drive(frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, double distance, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx);
void liftSwitch(frc::SpeedControllerGroup& lift);
void liftScale(frc::SpeedControllerGroup& lift);
void turn(frc::DifferentialDrive& drive, double angle, double kt, AHRS* navx);
void drop(frc::SpeedControllerGroup& intake);

void mobility(frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx);

void leftSwitch(frc::DifferentialDrive& drive, double kt, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx);
void rightSwitch(frc::DifferentialDrive& drive, double kt, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx);
void leftScale(frc::DifferentialDrive& drive, double kt, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx);
void rightScale(frc::DifferentialDrive& drive, double kt, frc::PIDController& leftController, frc::PIDController& rightController, frc::SpeedControllerGroup& lMotors, frc::SpeedControllerGroup& rMotors, frc::SpeedControllerGroup& lift, frc::SpeedControllerGroup& intake, frc::Encoder& leftEnc, frc::Encoder& rightEnc, AHRS* navx);

#endif
