// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface PivotIO{
    @AutoLog
    public static class PivotIOInputs {
        public Rotation2d position = new Rotation2d();
        public Rotation2d offsetPosition = new Rotation2d();
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs){}

    public default void setDesiredAngle(double angle){}
}
