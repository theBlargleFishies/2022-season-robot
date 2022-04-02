// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Test comment

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/Solenoid.h>
#include <frc/Compressor.h>
#include "frc/PneumaticsModuleType.h"
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Timer.h>
#include <frc/ADXRS450_Gyro.h>
#include "ctre/Phoenix.h"
#include <frc/Servo.h>
#include <tgmath.h>



class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom1 = "Back_it_up";
  const std::string kAutoNameCustom2 = "Combo";
  const std::string kAutoNameCustom3 = "High shoot";
  const std::string kAutoNameCustom4 = "Right Turn;";

  std::string m_autoSelected;

//-------------------------------------Pneumatics Objects-------------------------------------------------
  frc::PneumaticsModuleType CTREPCM;
  frc::Compressor comp {1, CTREPCM};
  frc::Solenoid Solenoid1 {1, CTREPCM, 0};
  frc::Solenoid Solenoid2 {1, CTREPCM,1}; 
  frc::Solenoid Solenoid3 {1, CTREPCM, 2};
  frc::Solenoid Solenoid4 {1, CTREPCM, 3}; /*<-- Make sure, when you uncomment this, that the second solenoid is plugged into port 1 of the PCM. 
                                              Don't forget to uncomment all of the Solenoid2's in the cpp and make sure the inputs are correct (true or false) 
                                              depending on what we are trying to do. Also, make sure I didn't miss any. Where ever there is a Solenoid1 there should
                                              also be a Solenoid2*/
  frc::Solenoid Solenoid5 {1, CTREPCM, 4}; // highshooter
//---------------------------------Motor Controller Related Objects--------------------------------------
  VictorSPX intake {7};

  rev::CANSparkMax frontLeft{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backLeft{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRight{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax backRight{5, rev::CANSparkMax::MotorType::kBrushless};
  
  rev::SparkMaxRelativeEncoder LeadRight = frontRight.GetEncoder();
  rev::SparkMaxRelativeEncoder LeadLeft = frontLeft.GetEncoder();

  rev::CANSparkMax::IdleMode kBrake;
  rev::CANSparkMax::IdleMode kCoast;

  frc::DifferentialDrive n_drive{frontLeft, frontRight};

//---------------------------------------Misc Objects--------------------------------------------------
  frc::Timer timer;
  frc::XboxController controller{1};
  frc::XboxController controller2{0};
  frc::ADXRS450_Gyro	Gyro;

//-----------------------------------Xbox Controller Variables-------------------------------------------
  double leftJoyStk_x;
  double leftJoyStk_y;
  double rightJoyStk_x;
  double rightJoyStk_y;
  bool rightStickB;


  double leftStickAdjX;
  double leftStickAdjY;
  double RightStickAdjX;
  double RightStickAdjY;

  bool Button_A = false;
  bool Button_B = false;
  bool Button_X = false;
  bool Button_Y = false;

  double leftTrigger = 0.0;
  double rightTrigger = 0.0;
  bool leftBumper = false;
  bool rightBumper = false;

  bool StartButton = false;
//---------------------------------Drive System Related Variables----------------------------------------
  double encoderROT1 = 0.0;
  double encoderROT2 = 0.0;
  double prevEncROT1 = 0.0;
  double prevEncROT2 = 0.0;
  double encDiff1 = 0.0;
  double encDiff2 = 0.0;
  double ForwardDrive = 0.0;
  double TurnDrive = 0.0;

  double driveRamp = 0;
  double turnRamp = 0;

  double rampFactor = 1;
  double rampOffset = 0;

//-------------------------------------Arm Related Variables---------------------------------------------
  bool prevBump = false; 
  bool armToggle = false; 
  bool armUp = false;
  bool prevB = false;
  bool inToggle = false;
//---------------------------------------Autonomous States-----------------------------------------------
  enum STATES{FORWARD,HISHOOT,BACKWARD,BAKCWARD2,TURN,SHOOT,INTAKE,STOP,LOWERARM,ARMUP};
  STATES states;
  STATES prvState;


//-----------------------------------------Misc Variables------------------------------------------------
  units::second_t startTime;
  units::second_t timeDiff = 0_s;
  double angle = 0.0;
  int count = 0;
  double temp = 0;
  double temp2 = 0;
};