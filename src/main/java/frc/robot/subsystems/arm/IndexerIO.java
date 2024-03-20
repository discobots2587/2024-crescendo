// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public Rotation2d position = new Rotation2d();
        public Rotation2d offsetPosition = new Rotation2d();
        public boolean beambreak = false;
        public boolean stowSwitch = false;
        public boolean deployedSwitch = false;
        public double indexerAppliedVolts = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double indexerCurrent = 0.0;
        public double hoodCurrent = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs){}

    default void setDesiredAngle(double desiredAngle){}

    default void setIndexerSpeed(double speed){}

    default void stop(){}

    default void stopHood(){}

    default void deployHood(){}

    default void stowHood(){}

    default void hoodPeriodic(){}
}
