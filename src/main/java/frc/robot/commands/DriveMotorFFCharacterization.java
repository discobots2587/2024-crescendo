// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveMotorFFCharacterization extends SequentialCommandGroup {
  /** Creates a new DriveMotorFFCharacterization. */
  public DriveMotorFFCharacterization(DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(
        () -> drive.setModuleStates(new SwerveModuleState[]{ 
                                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), 
                                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), 
                                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45 + 180)), 
                                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45 + 180))})),
      new WaitUntilCommand(() -> drive.atDesiredAngle()), // new WaitCommand(2),
      drive.getDriveQuasistaticSysId(Direction.kForward),
      drive.getDriveQuasistaticSysId(Direction.kReverse),
      drive.getDriveDynamicSysId(Direction.kForward),
      drive.getDriveDynamicSysId(Direction.kReverse)
    );
  }
}
