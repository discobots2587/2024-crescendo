// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    FlywheelIO io;
    FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Shooter(int masterID, int slaveID)
    {
        io = new SparkFlywheelIO(masterID, slaveID);
    }

    public void setDesiredVelocity(double desiredVelocity)
    {
        Logger.recordOutput("Shooter/setPointVelocity", desiredVelocity);
        io.setDesiredVelocity(desiredVelocity);
    }

    public void setDesiredVoltage(double desiredVoltage){
        Logger.recordOutput("Shooter/setPointVoltage", desiredVoltage);
        io.setDesiredVoltage(desiredVoltage);
    }

    public double getVelocity(){
        return inputs.velocityDeg;
    }

    public void setDesiredVoltage(double desiredVoltage)
    {
        SmartDashboard.putNumber("Target Voltage", desiredVoltage); //log the target voltage of the flywheel
        flywheelPID.setReference(desiredVoltage, CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void periodic(){}
}
