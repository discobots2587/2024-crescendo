// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


/** Add your docs here. */
public class VictorIntakeIO implements IntakeIO{
    VictorSPX intakeVictor;

    public VictorIntakeIO(int intakeID){
        intakeVictor = new VictorSPX(intakeID);

        intakeVictor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void setSpeed(double speed) {
        intakeVictor.set(VictorSPXControlMode.Velocity, speed);
    }

    @Override
    public void setVoltage(double volts) {
        intakeVictor.set(VictorSPXControlMode.PercentOutput, volts/12.0);
    }

    @Override
    public void stopMotor() {
        intakeVictor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
    }
}
