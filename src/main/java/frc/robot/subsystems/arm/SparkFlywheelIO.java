// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.FlywheelConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class SparkFlywheelIO implements FlywheelIO {
    private final CANSparkMax MasterSpark;
    private final CANSparkMax SlaveSpark;

    private final RelativeEncoder fwEnc;

    private final SparkPIDController flywheelPID;

    public SparkFlywheelIO(int masterID, int slaveID) {
        MasterSpark = new CANSparkMax(masterID, MotorType.kBrushless);
        SlaveSpark = new CANSparkMax(slaveID, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        MasterSpark.restoreFactoryDefaults();
        SlaveSpark.restoreFactoryDefaults();

        
        // Setup encoders and PID controller for the pivot sparkmax.
        fwEnc = MasterSpark.getEncoder();
        flywheelPID = MasterSpark.getPIDController();
        flywheelPID.setFeedbackDevice(fwEnc);
        // flywheelPID.

        // Apply position and velocity conversion factors for the turning encoder.
        // fwEnc.setVelocityConversionFactor(FlywheelConstants.kTurningEncoderVelocityFactor);

        
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


        // SlaveSpark.setInverted(true);
        SlaveSpark.follow(MasterSpark, true);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        MasterSpark.burnFlash();
        SlaveSpark.burnFlash();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs){
        inputs.positionDeg = fwEnc.getPosition();
        inputs.velocityDeg = fwEnc.getVelocity();
        inputs.masterAppliedVolts = MasterSpark.getAppliedOutput() * MasterSpark.getBusVoltage();
        inputs.slaveAppliedVolts = SlaveSpark.getAppliedOutput() * SlaveSpark.getBusVoltage();
        inputs.masterCurrentAmps = MasterSpark.getOutputCurrent();
        inputs.slaveCurrentAmps = SlaveSpark.getOutputCurrent();
    }

    @Override
    public void setDesiredVoltage(double voltage){
        flywheelPID.setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void setDesiredVelocity(double desiredVelocity){
        flywheelPID.setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void stop()
    {
        MasterSpark.stopMotor();
        SlaveSpark.stopMotor();    

        // fwEnc.getVelocity();
    }
}
