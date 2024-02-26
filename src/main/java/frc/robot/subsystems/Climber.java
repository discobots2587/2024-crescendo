// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Climber extends SubsystemBase{
    private final TalonFX LeftClimber;
    private final TalonFX RightClimber;

    private final DigitalInput leftHoming;
    private final DigitalInput rightHoming;

    public Climber(int leftID, int rightID, int leftHomingPort, int rightHomingPort)
    {
        LeftClimber = new TalonFX(leftID, "rio");
        RightClimber = new TalonFX(rightID, "rio");

        // Setup encoders and PID controller for the pivot sparkmax.
        leftHoming = new DigitalInput(leftHomingPort);
        rightHoming = new DigitalInput(rightHomingPort);

        LeftClimber.getConfigurator().apply(null);
        
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

    public boolean isLeftHomed()
    {
        return leftHoming.get(); // returns in degrees
    }

    public boolean isRightHomed()
    {
        return rightHoming.get(); // returns in degrees
    }

    public double getLeftLocation()
    {
        return LeftClimber.getRotorPosition(); // returms in rotations
    }

    public void setDesiredAngle(double desiredDegrees)
    {        
        PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        double desiredRotations = desiredDegrees/360.0;

        SmartDashboard.putNumber("Target Position", desiredRotations); 

        LeftClimber.setControl(m_request.withPosition(desiredRotations));
    }

    @Override
    public void periodic()
    {
        
    }
}
