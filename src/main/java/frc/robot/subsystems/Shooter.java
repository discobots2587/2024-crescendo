// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class Shooter extends SubsystemBase{
    private final CANSparkMax MasterSpark;
    private final CANSparkMax SlaveSpark;

    private final AbsoluteEncoder fwEnc;

    private final SparkPIDController flywheelPID;


    public Shooter(int masterID, int slaveID)
    {
        MasterSpark = new CANSparkMax(masterID, MotorType.kBrushless);
        SlaveSpark = new CANSparkMax(slaveID, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        MasterSpark.restoreFactoryDefaults();
        SlaveSpark.restoreFactoryDefaults();

        
        // Setup encoders and PID controller for the pivot sparkmax.
        fwEnc = MasterSpark.getAbsoluteEncoder(Type.kDutyCycle);
        flywheelPID = MasterSpark.getPIDController();
        flywheelPID.setFeedbackDevice(fwEnc);
        // flywheelPID.

        // Apply position and velocity conversion factors for the turning encoder.
        fwEnc.setVelocityConversionFactor(FlywheelConstants.kTurningEncoderVelocityFactor);

        
        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        flywheelPID.setP(FlywheelConstants.kP);
        flywheelPID.setI(FlywheelConstants.kI);
        flywheelPID.setD(FlywheelConstants.kD);
        flywheelPID.setFF(FlywheelConstants.kFF);
        flywheelPID.setOutputRange(FlywheelConstants.kVelocityMinOutput, FlywheelConstants.kVelocityMaxOutput);
        
        SlaveSpark.setIdleMode(FlywheelConstants.kFlywheelIdleMode);
        MasterSpark.setIdleMode(FlywheelConstants.kFlywheelIdleMode);

        MasterSpark.setSmartCurrentLimit(FlywheelConstants.kMotorCurrentLimit);
        SlaveSpark.setSmartCurrentLimit(FlywheelConstants.kMotorCurrentLimit);


        SlaveSpark.setInverted(true);
        SlaveSpark.follow(MasterSpark);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        MasterSpark.burnFlash();
        SlaveSpark.burnFlash();
    }



    public void setDesiredVelocity(double desiredVelocity)
    {
        SmartDashboard.putNumber("Target Velocity", desiredVelocity); //log the target vlocity of the flywheel
        flywheelPID.setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Flywheel Velocity", 0); // log the Velocity of the flywheel.
    }
}
