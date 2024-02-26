// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase{
    private final TalonFX LeftClimber;
    private final TalonFX RightClimber;

    private final DigitalInput leftHoming;
    private final DigitalInput rightHoming;

    private final PositionVoltage m_request;

    public Climber(int leftID, int rightID, int leftHomingPort, int rightHomingPort)
    {

        final TalonFXConfiguration ClimberConfigs = new TalonFXConfiguration();
        
        ClimberConfigs.Slot0.kP = ClimberConstants.kP;
        ClimberConfigs.Slot0.kI = ClimberConstants.kI;
        ClimberConfigs.Slot0.kD = ClimberConstants.kD;

        LeftClimber = new TalonFX(leftID, "rio");
        RightClimber = new TalonFX(rightID, "rio");

        // Setup encoders and PID controller for the pivot sparkmax.
        leftHoming = new DigitalInput(leftHomingPort);
        rightHoming = new DigitalInput(rightHomingPort);

        LeftClimber.getConfigurator().apply(ClimberConfigs);
        RightClimber.getConfigurator().apply(ClimberConfigs);

        m_request = new PositionVoltage(0).withSlot(0);
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
        return LeftClimber.getPosition().getValue(); // returms in rotations
    }

    public double getRightLocation()
    {
        return RightClimber.getPosition().getValue(); // returms in rotations
    }

    //Zeros climber;
    public void zeroLeft()
    {
        LeftClimber.setPosition(0);
    }

    public void zeroRight()
    {
        RightClimber.setPosition(0);
    }


    //Moving the climber
    public void setLeftDesiredPosition(double desiredDegrees)
    {        
        double desiredRotations = desiredDegrees/360.0;

        SmartDashboard.putNumber("Target Position", desiredRotations); 

        LeftClimber.setControl(m_request.withPosition(desiredRotations).withLimitReverseMotion(isLeftHomed()));
    }

    public void setRightDesiredPosition(double desiredDegrees)
    {        
        double desiredRotations = desiredDegrees/360.0;

        SmartDashboard.putNumber("Target Position", desiredRotations); 

        RightClimber.setControl(m_request.withPosition(desiredRotations).withLimitReverseMotion(isRightHomed()));
    }

    public void stopLeft()
    {LeftClimber.stopMotor();}

    public void stopRight()
    {RightClimber.stopMotor();}

    @Override
    public void periodic()
    {
        if(isLeftHomed())
        {zeroLeft();}
        if(isRightHomed())
        {zeroRight();}
    }
}
