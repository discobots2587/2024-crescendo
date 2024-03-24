// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
    private final CANSparkMax IntakeNEO;
    private IntakeConstants.IntakeState intakeState;
    // private double speed; 
    
    public Intake(int intakeID) {
        IntakeNEO = new CANSparkMax(intakeID, MotorType.kBrushless); //Brushed

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        IntakeNEO.restoreFactoryDefaults();

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.

        IntakeNEO.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        IntakeNEO.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        IntakeNEO.burnFlash();

        intakeState = IntakeConstants.IntakeState.STOPPED;
        // speed = 0;

    }

    public IntakeConstants.IntakeState getState()
    {
        return intakeState;
    }

    public void setSpeed(double speed)
    {
        IntakeNEO.set(speed);
        // this.speed = speed;
    }

    public void setVoltage(double volts){
        IntakeNEO.setVoltage(volts);
    }

    public void intake()
    {
        setVoltage(IntakeConstants.kIntakeVoltage);
        intakeState = IntakeConstants.IntakeState.INTAKING;
    }

    public void outtake()
    {
        setVoltage(IntakeConstants.kOuttakeVoltage);
        intakeState = IntakeConstants.IntakeState.OUTTAKING;
    }

    public void stop()
    {
        IntakeNEO.stopMotor();
        intakeState = IntakeConstants.IntakeState.STOPPED;
    }

    // @Override
    // public void periodic()
    // {
    //     SmartDashboard.putNumber("intake target speed", this.speed);
    // }
}
