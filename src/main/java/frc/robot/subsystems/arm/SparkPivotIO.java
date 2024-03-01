// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.PivotConstants;

import com.revrobotics.CANSparkLowLevel;

/** Add your docs here. */
public class SparkPivotIO implements PivotIO{
    private final CANSparkMax pivotSpark;

    private final AbsoluteEncoder absEncoder;
    
    private final SparkPIDController pivotPID;

    private final double angleOffset;

    public SparkPivotIO(int sparkID, double offset){
        pivotSpark = new CANSparkMax(sparkID, CANSparkLowLevel.MotorType.kBrushless);
        this.angleOffset = offset;

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        pivotSpark.restoreFactoryDefaults();

        // Setup encoder and PID Controller for the pivot SparkMax
        absEncoder = pivotSpark.getAbsoluteEncoder();
        pivotPID = pivotSpark.getPIDController();
        pivotPID.setFeedbackDevice(absEncoder);

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

        pivotSpark.setIdleMode(PivotConstants.kPivotIdleMode);
        pivotSpark.setSmartCurrentLimit(PivotConstants.kMotorCurrentLimit);

        pivotSpark.burnFlash();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs){
        inputs.position = Rotation2d.fromDegrees(absEncoder.getPosition());
        inputs.offsetPosition = Rotation2d.fromDegrees(absEncoder.getPosition() - angleOffset);
        inputs.appliedVolts = pivotSpark.getAppliedOutput() * pivotSpark.getBusVoltage();
        inputs.currentAmps = pivotSpark.getOutputCurrent();
    }

    @Override
    public void setDesiredAngle(double angle){
        double absDesiredAngle = angle + angleOffset;
        Logger.recordOutput("Arm/Pivot/SetPoint", Rotation2d.fromDegrees(absDesiredAngle));
        pivotPID.setReference(absDesiredAngle, CANSparkMax.ControlType.kPosition);
    }

}
