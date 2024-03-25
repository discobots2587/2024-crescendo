// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double positionDeg = 0.0;
        public double velocityDeg = 0.0;
        public double masterAppliedVolts = 0.0;
        public double slaveAppliedVolts = 0.0;
        public double masterCurrentAmps = 0.0;
        public double slaveCurrentAmps = 0.0;
        // public double[] currentAmps = new double[] {};
    }

    public default void updateInputs(FlywheelIOInputs inputs){}

    public default double getVelocity(){ 
        return 0.0; 
    }

    public default double getPosition(){ 
        return 0.0; 
    }

    public default void setDesiredVoltage(double voltage)
    {
        
    }

    public default void setDesiredVelocity(double desiredVelocity){}

    public default void stop(){}
}
