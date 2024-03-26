// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private IntakeConstants.IntakeState intakeState;
    // private double speed; 
    
    public Intake(int intakeID) {
        io = new SparkIntakeIO(intakeID);
        intakeState = IntakeConstants.IntakeState.STOPPED;
        // speed = 0;

    }

    public IntakeConstants.IntakeState getState()
    {
        return intakeState;
    }

    public void setSpeed(double speed)
    {
        io.setSpeed(speed);
        // this.speed = speed;
    }

    public void setVoltage(double volts){
        io.setVoltage(volts);
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
        io.stopMotor();
        intakeState = IntakeConstants.IntakeState.STOPPED;
    }

    // @Override
    // public void periodic()
    // {
    //     SmartDashboard.putNumber("intake target speed", this.speed);
    // }
}
