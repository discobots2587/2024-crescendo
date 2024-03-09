// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class IntakeIndex extends Command
{
    private BooleanSupplier intakeSup;
    
  /** Creates a new IntakeIndexRun. */
  public IntakeIndex(BooleanSupplier intakeBumper)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSup = intakeBumper;
    addRequirements(RobotContainer.arm);
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    RobotContainer.intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(intakeSup.getAsBoolean())
    {
      switch(RobotContainer.arm.getState())
      {
        case INTAKE:
          RobotContainer.intake.intake();
          RobotContainer.arm.load();// This method should spin the indexer until the beam break detects a break Then it hsould stop
          break;
        case AMP:
          RobotContainer.intake.stop();
          RobotContainer.arm.ampMode();
          break;
        case SHOOTER://The arm will be tracking the goal or at a preset, non-intake friendly angle.
          RobotContainer.intake.stop();
          RobotContainer.arm.shoot();
          break;
      }

    }
    else
    {
      RobotContainer.intake.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    RobotContainer.arm.intakeMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}