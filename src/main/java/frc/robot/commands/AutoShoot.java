// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoShoot extends Command
{

  private double time;

  public AutoShoot()
  {
    //Subsystem dependencies.
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
    //Spins up shooter
    RobotContainer.arm.setFlywheelVoltage(12);
    
    //Feeds note to shooter after 1 second has passed
    if(Timer.getFPGATimestamp() > time + 1.0) {
      RobotContainer.arm.shoot();
    }
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted)
  {
    RobotContainer.arm.stopFlywheel();      //Stops shooter
    RobotContainer.arm.indexStop();         //Stops indexer
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Stops command after 2 seconds have passed
    if(Timer.getFPGATimestamp() > time + 2.0) {
      return true;
    } else {
      return false;
    }
  }
}