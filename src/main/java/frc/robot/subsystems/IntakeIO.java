package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        double appliedVolts = 0.0;
        double currentAmps = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs){}

    default void setSpeed(double speed){}

    default void setVoltage(double volts){}

    default void stopMotor(){}
}
