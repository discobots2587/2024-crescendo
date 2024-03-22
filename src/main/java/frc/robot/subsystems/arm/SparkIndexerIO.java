// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IndexerConstants;

/** Add your docs here. */
public class SparkIndexerIO implements IndexerIO
{
    private final CANSparkMax indexerSpark;
    private final CANSparkMax hoodSpark;

    private final AbsoluteEncoder hoodEnc;
    private final SparkPIDController hoodPID;

    private final DigitalInput beambreakDigitalInput;
    private final DigitalInput stowSwitch;
    private final DigitalInput deployedSwitch;

    private final double angleOffset;

    public SparkIndexerIO(int beamBreakChannel, int indexerID, int hoodID, double offset){
        indexerSpark = new CANSparkMax(indexerID, MotorType.kBrushless);
        hoodSpark = new CANSparkMax(hoodID, MotorType.kBrushless);

        angleOffset = offset;

        beambreakDigitalInput = new DigitalInput(beamBreakChannel);
        stowSwitch = new DigitalInput(IndexerConstants.kStowID);
        deployedSwitch = new DigitalInput(IndexerConstants.kDeployedID);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        indexerSpark.restoreFactoryDefaults();
        hoodSpark.restoreFactoryDefaults();

        // Setup encoders and PID controller for the pivot sparkmax.
        hoodEnc = hoodSpark.getAbsoluteEncoder(Type.kDutyCycle);
        hoodPID = hoodSpark.getPIDController();
        hoodPID.setFeedbackDevice(hoodEnc);
        // hoodPID.

        // Apply position and velocity conversion factors for the turning encoder.
        hoodEnc.setPositionConversionFactor(IndexerConstants.kTurningEncoderPositionFactor);
        hoodEnc.setVelocityConversionFactor(IndexerConstants.kTurningEncoderVelocityFactor);

        
        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        hoodPID.setP(IndexerConstants.kP);
        hoodPID.setI(IndexerConstants.kI);
        hoodPID.setD(IndexerConstants.kD);
        hoodPID.setFF(IndexerConstants.kFF);
        hoodPID.setOutputRange(IndexerConstants.kVelocityMinOutput, IndexerConstants.kVelocityMaxOutput);
        
        hoodSpark.setIdleMode(IndexerConstants.kHoodIdleMode);
        indexerSpark.setIdleMode(IndexerConstants.kIndexerIdleMode);

        hoodSpark.setSmartCurrentLimit(IndexerConstants.kHoodCurrentLimit);
        indexerSpark.setSmartCurrentLimit(IndexerConstants.kIndexerCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        hoodSpark.burnFlash();
        indexerSpark.burnFlash();
    }

    @Override
    public void setDesiredAngle(double desiredAngle) {
        hoodPID.setReference(desiredAngle - angleOffset, CANSparkMax.ControlType.kPosition);
        Logger.recordOutput("Indexer/Hood/SetPoint", desiredAngle - angleOffset);
    }

    @Override
    public void setIndexerSpeed(double speed){
        indexerSpark.set(speed);
        Logger.recordOutput("Indexer/SetPoint", speed);
    }

    @Override
    public void stop() {
        indexerSpark.stopMotor();
    }

    @Override
    public void stopHood(){
        hoodSpark.stopMotor();
    }

    @Override
    public void stowHood(){
        if(!stowSwitch.get()){
            hoodSpark.stopMotor();
        } else {
            hoodSpark.setVoltage(-5.0);
        }
    }

    @Override
    public void deployHood(){
        if(!deployedSwitch.get()){
            hoodSpark.stopMotor();
        } else {
            hoodSpark.setVoltage(5.0);
        }
    }

    @Override
    public void hoodPeriodic(){
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.position = Rotation2d.fromDegrees(hoodEnc.getPosition());
        inputs.offsetPosition = inputs.position.minus(Rotation2d.fromDegrees(angleOffset));
        inputs.beambreak = beambreakDigitalInput.get();
        inputs.stowSwitch = stowSwitch.get();
        inputs.deployedSwitch = deployedSwitch.get();
        inputs.indexerAppliedVolts = indexerSpark.getAppliedOutput() * indexerSpark.getBusVoltage();
        inputs.hoodAppliedVolts = hoodSpark.getAppliedOutput() * hoodSpark.getBusVoltage();
        inputs.indexerCurrent = indexerSpark.getOutputCurrent();
        inputs.hoodCurrent = hoodSpark.getOutputCurrent();
    }
    
}
