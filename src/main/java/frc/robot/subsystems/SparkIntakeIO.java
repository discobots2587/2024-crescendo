// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class SparkIntakeIO implements IntakeIO{
    private final CANSparkMax intakeSpark;

    public SparkIntakeIO(int intakeID){
        intakeSpark = new CANSparkMax(intakeID, MotorType.kBrushless); //Brushed

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        intakeSpark.restoreFactoryDefaults();


        // Reduce frequency of signal for unneeded status signals (velocity, position, etc) to reduce CAN usage
        intakeSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 32767); 
        intakeSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 32767);
        intakeSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 32767);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.

        intakeSpark.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        intakeSpark.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        intakeSpark.burnFlash();
        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.appliedVolts = intakeSpark.getAppliedOutput() * intakeSpark.getBusVoltage();
        inputs.currentAmps = intakeSpark.getOutputCurrent();
    }

    @Override
    public void setSpeed(double speed) {
        intakeSpark.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        intakeSpark.setVoltage(volts);
    }

    @Override
    public void stopMotor(){
        intakeSpark.stopMotor();
    }
    
}
