// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IntakeConstants;


public class Intake {
    private final CANSparkMax IntakeNEO;
    private int intakeState;
    /* 
     * 1 = intake
     * 0 = stopped
     * -1 = outtake
    */
    
    public Intake(int intakeID) {
        IntakeNEO = new CANSparkMax(intakeID, MotorType.kBrushless);

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

        intakeState = 0;
    }

    public int getState()
    {
        return intakeState;
    }

    public void setSpeed(double speed)
    {
        IntakeNEO.set(speed);
    }

    public void intake()
    {
        setSpeed(IntakeConstants.kIntakeSpeed);
        intakeState = 1;
    }

    public void outtake()
    {
        setSpeed(IntakeConstants.kOuttakeSpeed);
        intakeState = -1;
    }

    public void stop()
    {
        IntakeNEO.stopMotor();
        intakeState = 0;
    }
}
