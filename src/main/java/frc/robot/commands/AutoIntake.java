// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoIntake extends Command
{
  private double time;

  public AutoIntake()
  {
    //Subsystem dependencies
    addRequirements(RobotContainer.arm);
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    //Timestamp in which the command starts
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    RobotContainer.intake.intake();     //Intake note
    RobotContainer.arm.load();          //Stores note in indexer 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    RobotContainer.intake.stop();     //Stops intake
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Stops command after 5 seconds have passed
    if(Timer.getFPGATimestamp() > time + 5.0) {
        return true;
      } else {
        return false;
      }
    }
}