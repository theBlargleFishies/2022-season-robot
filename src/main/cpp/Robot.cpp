// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TEST comment

#include "Robot.h"
#include <fmt/core.h>
#include <stdio.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom1, kAutoNameCustom1);
  m_chooser.AddOption(kAutoNameCustom2, kAutoNameCustom2);
  m_chooser.AddOption(kAutoNameCustom3, kAutoNameCustom3);
  m_chooser.AddOption(kAutoNameCustom4, kAutoNameCustom4);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  frontLeft.RestoreFactoryDefaults();
  frontRight.RestoreFactoryDefaults();
  backLeft.RestoreFactoryDefaults();
  backRight.RestoreFactoryDefaults();

  
  backLeft.Follow(frontLeft); // setting the followers
  backRight.Follow(frontRight);

  frontLeft.SetOpenLoopRampRate(0);
  frontRight.SetOpenLoopRampRate(0);
  backLeft.SetOpenLoopRampRate(0);
  backRight.SetOpenLoopRampRate(0);

  armUp = true; // arm in raised position

  frc::CameraServer::StartAutomaticCapture();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

//----------------------------------Printing to Dashboard-------------------------------------------------
frc::SmartDashboard::PutNumber("Left Y :", leftJoyStk_y);
frc::SmartDashboard::PutNumber("Right X :", rightJoyStk_x);
frc::SmartDashboard::PutNumber("Left Trigger :", leftTrigger);
frc::SmartDashboard::PutBoolean("Right Bumper :", rightBumper);
frc::SmartDashboard::PutNumber("Right Encoder:", LeadRight.GetPosition());
frc::SmartDashboard::PutNumber("Left Encoder:", LeadLeft.GetPosition());
frc::SmartDashboard::PutNumber("Gyro Angle :", angle);
frc::SmartDashboard::PutNumber("Port :", Gyro.GetPort());
frc::SmartDashboard::PutBoolean("Arm Toggle", armToggle);
frc::SmartDashboard::PutBoolean("Arm1", Solenoid1.Get());
frc::SmartDashboard::PutNumber("forward drive", ForwardDrive);
frc::SmartDashboard::PutNumber("turn drive", TurnDrive);
frc::SmartDashboard::PutNumber("count", count);
frc::SmartDashboard::PutBoolean("intake Toggle", inToggle);
frc::SmartDashboard::PutBoolean("B", Button_B);
frc::SmartDashboard::PutNumber("Lead right", frontRight.Get());
frc::SmartDashboard::PutNumber("Lead Right", frontLeft.Get());
frc::SmartDashboard::PutNumber("Right after shoot", encoderROT1);
frc::SmartDashboard::PutNumber("Left after Shoot", encoderROT2);

angle = Gyro.GetAngle(); // updating the angle on the gyro
}


/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom1) 
  {
    states = BACKWARD; // set beginning state for this mode
  }
  
  else if (m_autoSelected == kAutoNameCustom2) 
  {
    states = SHOOT; // set beginning state for this mode
  }
  else if (m_autoSelected == kAutoNameDefault) 
  { 
    states = HISHOOT;
  }

  
  // Setting the modes on the CanSparkMax motors
  frontRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  frontLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  backRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  backLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  timer.Reset();
  timer.Start();
}

void Robot::AutonomousPeriodic() { 
  /* Test the first one to see if it will work, timer doesn't seem to work the way we think it does. I'll look into it more. 
  The first one should work. I renamed it to Back_it_up. Test it to make sure it backs up and stopps. Make sure soneone has their finger on the 
  diable button in case we have to flip our -7.5 and 7.5 */

  // back up and stop.
  count = 0;
  if (m_autoSelected == kAutoNameCustom1) // Back it up
  {
    switch (states)
    {
    case BACKWARD:
    while ((encoderROT1/ 6.82) > -8.5 && (encoderROT2/ 6.82) < 8.5) // might have to switch the 7.5's
      {
        n_drive.ArcadeDrive(0,0.7,true);
        encoderROT1 = LeadRight.GetPosition();
        encoderROT2 = LeadLeft.GetPosition();  
      }
       states = STOP; // update the state
      break;

      case STOP:
      n_drive.ArcadeDrive(0,0,true);
      break;
    }
  } 
  // Shoot, back up, and stop.
  else if (m_autoSelected == kAutoNameCustom2) // Combo
  {
    switch (states)
    {
      case SHOOT:


      prevEncROT1 = LeadRight.GetPosition();
      prevEncROT2 = LeadLeft.GetPosition();  


      if(timer.Get() < 2_s)
      {

           intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
      }
      else{
          states = BACKWARD;
          } // update the state
          break;

      case BACKWARD:

        while ((encoderROT1/ 6.82) > -8 && (encoderROT2/ 6.82) < 8) // might have to switch the 7.5's
        {
          n_drive.ArcadeDrive(0,0.5);
          encoderROT1 = LeadRight.GetPosition();
          encoderROT2 = LeadLeft.GetPosition();
        }
        states = STOP; // ubdate the state
          intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
          break;
      
      case STOP:
          n_drive.ArcadeDrive(0,0,true);
          break;
    }
     
  }
 
  else if (m_autoSelected == kAutoNameDefault) //High Shoot
  {
   switch (states) 
  {

    case HISHOOT:

      if (LeadRight.GetPosition() > 0 || LeadRight.GetPosition() < 0)
      {
        LeadRight.SetPosition(0);
        LeadLeft.SetPosition(0);
      }
      
      Solenoid5.Set(true);

      states = LOWERARM;
      prvState = HISHOOT;
      timer.Reset();

      break;


    case LOWERARM:
   
    if (timer.Get() < 1_s)
    {
    Solenoid1.Set(true);
    Solenoid2.Set(false);
    Solenoid3.Set(true);
    Solenoid4.Set(false);
    }

    else{
      states = FORWARD;
      prvState = LOWERARM;


      LeadRight.SetPosition(0);
      LeadLeft.SetPosition(0);

      prevEncROT1 = LeadRight.GetPosition();
      prevEncROT2 = LeadLeft.GetPosition();

      timer.Reset();
    }

    break;

    
    case FORWARD:
      if(prvState == LOWERARM)
      {
        while((abs (encDiff1 / 0.568) < 35) && (abs (encDiff2 / .568) < 35))  
        {
          frontRight.Set(0.4);
          frontLeft.Set(-0.4);
          encDiff1 = LeadRight.GetPosition() - prevEncROT1;
          encDiff2 = LeadLeft.GetPosition() - prevEncROT2;
          intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
        }
        states = ARMUP;
      }
      if(prvState == TURN)
      {
        while ((abs (encDiff1 / 0.568) < 120) && (abs (encDiff2 / .568) < 120))  
        {
          frontRight.Set(0.4);
          frontLeft.Set(-0.4);
          encDiff1 = LeadRight.GetPosition() - prevEncROT1;
          encDiff2 = LeadLeft.GetPosition() - prevEncROT2;
        }
        timer.Reset();
        states = SHOOT;

      }
      prevEncROT1 = LeadRight.GetPosition();
      prevEncROT2 = LeadLeft.GetPosition();
      timer.Reset();
      break;
     
    case ARMUP:
    wpi::outs() << "ARM UP!";
      if(timer.Get() < 1_s)
      {
        intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        Solenoid1.Set(false);
        Solenoid2.Set(true);
        Solenoid3.Set(false);
        Solenoid4.Set(true);
       
       }
        else 
        {
          states = TURN;
          prvState = ARMUP;
          prevEncROT1 = LeadRight.GetPosition();
          prevEncROT2 = LeadLeft.GetPosition();

          LeadRight.SetPosition(0);
          LeadLeft.SetPosition(0);

          Gyro.Reset();
        }

      break;

   case TURN:
    
    while( abs(LeadLeft.GetPosition() / .568) < abs (35.0) || abs(LeadRight.GetPosition() / .568) < abs (35.0))
    {
      frontLeft.Set(0.15); // TODO: swap left and right and fix logic
      frontRight.Set(0.15); // TODO: swap left and right and fix logic
      encDiff1 = LeadRight.GetPosition() - prevEncROT1;
      encDiff2 = LeadLeft.GetPosition() - prevEncROT2;
    }

    LeadRight.SetPosition(0);
    LeadLeft.SetPosition(0);

    timer.Reset();
    if(timer.Get() < 2_s)
      
    states=FORWARD;
    prvState=TURN;
    prevEncROT1 = LeadRight.GetPosition();
    prevEncROT2 = LeadLeft.GetPosition();

    break;

  case SHOOT:
      
      frc::Wait(1_s);
      intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
      frc::Wait(2_s);

        //update previous positions
        prevEncROT1 = LeadRight.GetPosition();
        prevEncROT2 = LeadLeft.GetPosition();
        encoderROT1 = LeadRight.GetPosition();
        encoderROT2 = LeadLeft.GetPosition();
        
        //upstate states
        states = BAKCWARD2;
        prvState = SHOOT;

      break;

  case BAKCWARD2:
        while (((LeadRight.GetPosition() / 0.568) > -5) && ((LeadLeft.GetPosition() / .568) < 5))  
        {
          intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0);
          frontRight.Set(-0.4);
          frontLeft.Set(0.4);
          encDiff1 = LeadRight.GetPosition() - prevEncROT1;
          encDiff2 = LeadLeft.GetPosition() - prevEncROT2;
        }
        prvState = BACKWARD;
        break;
      
    
   default:
      intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      frontRight.Set(0);
      frontLeft.Set(0);
      LeadLeft.SetPosition(0);
      LeadRight.SetPosition(0); 
     break;
   }
  }

  else if (m_autoSelected == kAutoNameCustom4)
  {
    
  }

  else
  {
    n_drive.TankDrive(0,0,true);
  }
}

void Robot::TeleopInit() {
  frontRight.SetIdleMode (rev::CANSparkMax::IdleMode::kBrake);
  frontLeft.SetIdleMode (rev::CANSparkMax::IdleMode::kBrake);
  backRight.SetIdleMode (rev::CANSparkMax::IdleMode::kBrake);
  backLeft.SetIdleMode (rev::CANSparkMax::IdleMode::kBrake);
  armToggle = true;

}

void Robot::TeleopPeriodic() {
//----------------------------------Xbox Controller Outputs--------------------------------------------
// bumpers and buttons
rightBumper = controller2.GetRightBumper();
leftBumper = controller2.GetLeftBumper();
rightStickB = controller2.GetBButton();
Button_X = controller2.GetXButton();
leftTrigger = controller.GetLeftTriggerAxis();

//-------------------------------------Arm Control-----------------------------------------------------
// Arm Toggle


//-------------------------------------Control Variables-----------------------------------------------

Solenoid5.Set(Button_X);

if (rightBumper && ! prevBump)
{
  if(armToggle)
  {
    armToggle = false;
  }                                            
  else{
    armToggle = true;
  }
}
prevBump = rightBumper;

if (rightStickB && ! prevB)
{
  if(inToggle)
  {
    inToggle = false;
  }
  else{
    inToggle = true;
  }
}
prevB = rightStickB;

//Arm up
if(armToggle)
{
  if(leftBumper)
  {
    intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1); // Outtake
  }
  else
  {
    intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0); // Stopped
  }
  Solenoid1.Set(false);
  Solenoid2.Set(true);
  Solenoid3.Set(false);
  Solenoid4.Set(true);

}

//Arm down
if(!armToggle)
{
  if(leftBumper)
  {
    intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1); // Outtake
  }
   else if(inToggle)
  {
    intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0); // Outtake
  }
  else
  {
    intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, .65); // Intake
  }
  Solenoid1.Set(true);
  Solenoid2.Set(false);
  Solenoid3.Set(true);
  Solenoid4.Set(false);
}

//-------------------------------------Drive Control-----------------------------------------------------
// Forward/Backward Acceleration
// Joysticks
if (armToggle)
{
  frontLeft.SetOpenLoopRampRate(1);
  frontRight.SetOpenLoopRampRate(1);
  backLeft.SetOpenLoopRampRate(1);
  backRight.SetOpenLoopRampRate(1);
}
else
{
  frontLeft.SetOpenLoopRampRate(0.3);
  frontRight.SetOpenLoopRampRate(0.3);
  backLeft.SetOpenLoopRampRate(0.3);
  backRight.SetOpenLoopRampRate(0.3);
}
n_drive.ArcadeDrive( RightStickAdjX, leftStickAdjY, true);

}

//---------------------------------------------------------------------------------------------------
void Robot::DisabledInit() {
  armUp = true; // Don't really need this, but just in case.
  //armToggle = false; // reset arm toggle so arm is always up, this might have to be changed to true when pneumatics are switched.
  frontRight.SetIdleMode (rev::CANSparkMax::IdleMode::kCoast);
  frontLeft.SetIdleMode (rev::CANSparkMax::IdleMode::kCoast);
  backRight.SetIdleMode (rev::CANSparkMax::IdleMode::kCoast);
  backLeft.SetIdleMode (rev::CANSparkMax::IdleMode::kCoast);
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif