// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class IntakeTest extends Command
{
    private DoubleSupplier intakeSup;
    private DoubleSupplier indexerSup;
    
  /** Creates a new IntakeIndexRun. */
  public IntakeTest(DoubleSupplier IntakeJoy, DoubleSupplier IndexerJoy)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSup = IntakeJoy;
    indexerSup = IndexerJoy;
    addRequirements(RobotContainer.arm);
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    RobotContainer.intake.outtake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if (Math.abs(intakeSup.getAsDouble()) > 0.05)
    {
        RobotContainer.intake.setSpeed(intakeSup.getAsDouble());
        // SmartDashboard.putNumber("intakespeed", intakeSup.getAsDouble());
    }
    else
    {
        RobotContainer.intake.stop();
    }

    if(Math.abs(indexerSup.getAsDouble()) > 0.05)
    {
        RobotContainer.arm.indexTest(indexerSup.getAsDouble());
        // SmartDashboard.putNumber("indexspeed", intakeSup.getAsDouble());
    }
    else
    {
        RobotContainer.arm.indexStop();
    }

    SmartDashboard.putNumber("index speed", indexerSup.getAsDouble());
    SmartDashboard.putNumber("intake speed", intakeSup.getAsDouble());
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