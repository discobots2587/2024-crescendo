// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import frc.robot.Constants.PivotConstants;

public class Pivot {
    private final CANSparkMax MasterSpark;
    private final CANSparkMax SlaveSpark;

    private final AbsoluteEncoder absEncoder;

    private final SparkPIDController pivotPID;

    private double angleOffset; // This is the initial value of the arm. This shoukld be aiming upwards at about 60 degrees.
    private double m_desiredAngle;

    public Pivot(int masterID, int slaveID, double offset)
    {
        MasterSpark = new CANSparkMax(masterID, MotorType.kBrushless);
        SlaveSpark = new CANSparkMax(slaveID, MotorType.kBrushless);

        angleOffset = offset;
        m_desiredAngle = offset;

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        MasterSpark.restoreFactoryDefaults();
        SlaveSpark.restoreFactoryDefaults();

        
        // Setup encoders and PID controller for the pivot sparkmax.
        absEncoder = MasterSpark.getAbsoluteEncoder(Type.kDutyCycle);
        pivotPID = MasterSpark.getPIDController();
        pivotPID.setFeedbackDevice(absEncoder);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        absEncoder.setPositionConversionFactor(PivotConstants.kTurningEncoderPositionFactor);
        absEncoder.setVelocityConversionFactor(PivotConstants.kTurningEncoderVelocityFactor);

        
        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        pivotPID.setP(PivotConstants.kP);
        pivotPID.setI(PivotConstants.kI);
        pivotPID.setD(PivotConstants.kD);
        pivotPID.setFF(PivotConstants.kFF);
        pivotPID.setOutputRange(PivotConstants.kTurningMinOutput, PivotConstants.kTurningMaxOutput);
        
        SlaveSpark.setIdleMode(PivotConstants.kPivotIdleMode);
        MasterSpark.setIdleMode(PivotConstants.kPivotIdleMode);

        MasterSpark.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);
        SlaveSpark.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);


        SlaveSpark.setInverted(true);
        SlaveSpark.follow(MasterSpark);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        MasterSpark.burnFlash();
        SlaveSpark.burnFlash();
    }

    public double getRawPosition()
    {
        return absEncoder.getPosition(); // returns in degrees
    }

    public double getPointing()
    {
        return getRawPosition() - angleOffset; // returns in degrees
    }

    public double getDesiredState()
    {
        return m_desiredAngle;
    }


    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredAngle(double desiredAngle) {
        double turnPose = desiredAngle;

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        pivotPID.setReference(turnPose, CANSparkMax.ControlType.kPosition);

        m_desiredAngle = desiredAngle;
    }
}
