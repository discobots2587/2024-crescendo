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
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase{
    private final CANSparkMax MasterSpark;
    // private final CANSparkMax SlaveSpark;

    private final AbsoluteEncoder absEncoder;

    private final SparkPIDController pivotPID;

    private double angleOffset; // This is the initial value of the arm. This shoukld be aiming upwards at about 60 degrees.

    public Pivot(int masterID, double offset)
    {
        MasterSpark = new CANSparkMax(masterID, MotorType.kBrushless);
        // SlaveSpark = new CANSparkMax(slaveID, MotorType.kBrushless);

        angleOffset = offset;

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        MasterSpark.restoreFactoryDefaults();
        // SlaveSpark.restoreFactoryDefaults();

        
        // Setup encoders and PID controller for the pivot sparkmax.
        absEncoder = MasterSpark.getAbsoluteEncoder(Type.kDutyCycle);
        pivotPID = MasterSpark.getPIDController();
        pivotPID.setFeedbackDevice(absEncoder);
        // pivotPID.

        // Apply position and velocity conversion factors for the turning encoder.
        absEncoder.setPositionConversionFactor(PivotConstants.kTurningEncoderPositionFactor);
        absEncoder.setVelocityConversionFactor(PivotConstants.kTurningEncoderVelocityFactor);

        
        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        pivotPID.setP(PivotConstants.kP);
        pivotPID.setI(PivotConstants.kI);
        pivotPID.setD(PivotConstants.kD);
        pivotPID.setFF(PivotConstants.kFF);
        pivotPID.setOutputRange(PivotConstants.kTurningMinOutput, PivotConstants.kTurningMaxOutput);
        
        // SlaveSpark.setIdleMode(PivotConstants.kPivotIdleMode);
        MasterSpark.setIdleMode(PivotConstants.kPivotIdleMode);

        MasterSpark.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);
        // SlaveSpark.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);


        // SlaveSpark.setInverted(true);
        // SlaveSpark.follow(MasterSpark);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        MasterSpark.burnFlash();
        // SlaveSpark.burnFlash();
    }

    public double getRawPosition()
    {
        return absEncoder.getPosition(); // returns in degrees
    }

    public double getAiming()
    {
        return getRawPosition() - angleOffset; // returms in degrees
    }

    public void setDesiredAngle(double desiredAngle)
    {
        double turnPose = desiredAngle - angleOffset;

        SmartDashboard.putNumber("Target Position", turnPose); 
        pivotPID.setReference(turnPose, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Aiming Position", getAiming()); // log the aiming position of the arm.
    }
}
