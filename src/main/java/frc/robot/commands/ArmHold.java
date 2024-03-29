// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ArmHold extends Command
{
    private BooleanSupplier shootMode;
    private BooleanSupplier ampMode;

  /** Creates a new ArmHold. */
  public ArmHold(BooleanSupplier shooter, BooleanSupplier amp)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    shootMode = shooter;
    ampMode = amp;
    addRequirements(RobotContainer.arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(shootMode.getAsBoolean())
    {
        RobotContainer.arm.shooterMode(0);
    }
    else if (ampMode.getAsBoolean())
    {
        RobotContainer.arm.ampMode();
    }
    else
    {
        RobotContainer.arm.intakeMode();
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