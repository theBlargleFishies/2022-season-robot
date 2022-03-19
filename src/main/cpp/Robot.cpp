// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

  /*frontRight.SetIdleMode (kCoast);
  frontLeft.SetIdleMode (kCoast);
  backRight.SetIdleMode (kCoast);
  backLeft.SetIdleMode (kCoast);*/

  backLeft.Follow(frontLeft); // setting the followers
  backRight.Follow(frontRight);

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
frc::SmartDashboard::PutNumber("Right Trigger :", rightTrigger);
frc::SmartDashboard::PutBoolean("Right Bumper :", rightBumper);
frc::SmartDashboard::PutNumber("Right Encoder:", LeadRight.GetPosition());
frc::SmartDashboard::PutNumber("Left Encoder:", LeadLeft.GetPosition());
frc::SmartDashboard::PutNumber("Gyro Angle :", angle);
frc::SmartDashboard::PutNumber("Port :", Gyro.GetPort());
frc::SmartDashboard::PutBoolean("Arm Toggle", armToggle);
frc::SmartDashboard::PutBoolean("Arm1", Solenoid1.Get());
//frc::SmartDashboard::PutBoolean("Arm2", Solenoid2.Get());
frc::SmartDashboard::PutNumber("forward drive", ForwardDrive);
frc::SmartDashboard::PutNumber("turn drive", TurnDrive);
frc::SmartDashboard::PutNumber("count", count);
frc::SmartDashboard::PutBoolean("intake Toggle", inToggle);
frc::SmartDashboard::PutBoolean("B", Button_B);

angle = Gyro.GetAngle(); // updating the angle on the gyro

leftJoyStk_y = controller.GetLeftY();
rightJoyStk_x = controller.GetRightX();


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
  else if (m_autoSelected == kAutoNameCustom3) 
  { 
    states = LOWERARM;
  }
  //Setting inital encoder positions to 0, I think?
  LeadLeft.SetPosition(0);
  LeadRight.SetPosition(0); 
  
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
  Solenoid1.Set(false);
  Solenoid2.Set(true);
  Solenoid3.Set(false);
  Solenoid4.Set(true);
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
      if(timer.Get() < 2_s)
      {

           intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
      }
      else{
          states = BACKWARD;
          } // update the state
          break;

      case BACKWARD:

        while ((encoderROT1/ 6.82) > -6.5 && (encoderROT2/ 6.82) < 6.5) // might have to switch the 7.5's
        {
          n_drive.ArcadeDrive(0,0.5);
          encoderROT1 = LeadRight.GetPosition();
          encoderROT2 = LeadLeft.GetPosition();
        }
        states = STOP; // ubdate the state
   
          break;
      
      case STOP:
          n_drive.ArcadeDrive(0,0,true);
          break;
    }
     
  }
 
  else if (m_autoSelected == kAutoNameCustom3) //High Shoot
  {
   switch (states) 
  {
   case LOWERARM:
    if (timer.Get() < 2_s)
    {
    Solenoid1.Set(true);
    Solenoid2.Set(false);
    Solenoid3.Set(true);
    Solenoid4.Set(false);
    }

    else{
      states= FORWARD;
      prvState = LOWERARM;
    }

      break;

    case FORWARD:
     if(prvState == LOWERARM)
     {
      while((abs (encDiff1 / 0.568) < 39) && (abs (encDiff2 / .568) < 39))  
      {
        frontRight.Set(0.6);
        frontLeft.Set(-0.6);
        encDiff1 = LeadRight.GetPosition() - prevEncROT1;
        encDiff2 = LeadLeft.GetPosition() - prevEncROT2;
      }
     }
      else if(prvState == TURN)
      {
       while ((abs (encDiff1 / 0.568) < 99) && (abs (encDiff2 / .568) < 99))  
       {
        frontRight.Set(0.6);
        frontLeft.Set(-0.6);
        encDiff1 = LeadRight.GetPosition() - prevEncROT1;
        encDiff2 = LeadLeft.GetPosition() - prevEncROT2;
       }
      }
      else 
      {
       states = ARMUP;
       prvState = FORWARD;
       prevEncROT1 = LeadRight.GetPosition();
       prevEncROT2 = LeadLeft.GetPosition();
       timer.Reset();
      }
      break;
     
     case ARMUP:
      if(timer.Get() < 2_s)
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
          Gyro.Reset();
        }

      break;

   case TURN:
    while (Gyro.GetAngle() < 180)
    {
      frontLeft.Set(-0.5);
      frontRight.Set(0.5);
    }
    states=FORWARD;
    prvState=TURN;
    prevEncROT1 = LeadRight.GetPosition();
    prevEncROT2 = LeadLeft.GetPosition();
    
     break;

   case SHOOT:
      if(timer.Get() < 2_s)
      {
         intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
      }
      else{
          states = BACKWARD;
          } // update the state
          break;

      case BACKWARD:

        while ((abs (encDiff1 / 0.568) < 99) && (abs (encDiff2 / .568) < 99))  
          {
          n_drive.ArcadeDrive(0,0.5);
          encoderROT1 = LeadRight.GetPosition() - prevEncROT1;
          encoderROT2 = LeadLeft.GetPosition() - prevEncROT2;
        }
        states = STOP; // update the state
   
          break;
      
    
   default:
      intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      frontRight.Set(0);
      frontLeft.Set(0);
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
  frontRight.SetIdleMode (rev::CANSparkMax::IdleMode::kCoast);
  frontLeft.SetIdleMode (rev::CANSparkMax::IdleMode::kCoast);
  backRight.SetIdleMode (rev::CANSparkMax::IdleMode::kCoast);
  backLeft.SetIdleMode (rev::CANSparkMax::IdleMode::kCoast);
  armToggle = true;

}

void Robot::TeleopPeriodic() {
//----------------------------------Xbox Controller Outputs--------------------------------------------
// bumpers and buttons
rightBumper = controller2.GetRightBumper();
leftBumper = controller2.GetLeftBumper();
rightStickB = controller2.GetBButton();

//-------------------------------------Arm Control-----------------------------------------------------
// Arm Toggle
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
    intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -.6); // Outtake
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
    intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -.6); // Outtake
  }
   else if(inToggle)
  {
    intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0); // Outtake
  }
  else
  {
    intake.Set (ctre::phoenix::motorcontrol::ControlMode::PercentOutput, .6); // Intake
  }
  Solenoid1.Set(true);
  Solenoid2.Set(false);
  Solenoid3.Set(true);
  Solenoid4.Set(false);
}

//-------------------------------------Drive Control-----------------------------------------------------
// Forward/Backward Acceleration
// Joysticks
if (abs(controller.GetLeftY()) < 0.1 ) // might have to change this to 0.1. Try it and see if it's better.
{
  ForwardDrive = controller.GetLeftY();
}
else
{
  if (controller.GetLeftY() > ForwardDrive)
  {
    ForwardDrive += 0.01; // change this to change acceleration, bigger number = faster accel
  }
  if (controller.GetLeftY() < ForwardDrive)
  {
    ForwardDrive -= 0.01; // change this to change acceleration, bigger number = faster accel
  }
}

// Turning acceleration
if (abs(controller.GetRightX()) < 0.1 ) // might have to change this to 0.1. Try it and see if it's better.
{
  TurnDrive = controller.GetRightX();
}
else
{
 if (controller.GetRightX() > TurnDrive)
  {
    TurnDrive += 0.01;// change this to change acceleration, bigger number = faster accel
  }
 if (controller.GetRightX() < TurnDrive)
  {
    TurnDrive -= 0.01;// change this to change acceleration, bigger number = faster accel
  }
}

if(armToggle)
{
n_drive.ArcadeDrive(TurnDrive * 0.7 , ForwardDrive * 0.7, true); //Kinda like cod games
}
else{
n_drive.ArcadeDrive(TurnDrive , ForwardDrive, true); //Kinda like cod games
}

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