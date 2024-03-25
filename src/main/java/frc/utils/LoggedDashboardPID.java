// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.Arrays;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LoggedDashboardPID implements LoggedDashboardInput, Sendable{
    private final String key;
    private double[] defaultValue;
    private double[] value;

    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table){
            table.put(key + "kP", value[0]);
            table.put(key + "kI", value[1]);
            table.put(key + "kD", value[2]);
            table.put(key + "kIZone", value[3]);
        }

        public void fromLog(LogTable table){
            table.get(key + "kP", defaultValue[0]);
            table.get(key + "kI", defaultValue[1]);
            table.get(key + "kD", defaultValue[2]);
            table.get(key + "kIZone", defaultValue[3]);
        }
    };

    public LoggedDashboardPID(String key){
        this(key, new double[5]);
    }

    public LoggedDashboardPID(String key, double[] defaultValue){
        this.key = key;
        this.defaultValue = Arrays.copyOf(defaultValue, 5);
        this.value = Arrays.copyOf(defaultValue, 5);
        SmartDashboard.putData(key, this);
        periodic();
        Logger.registerDashboardInput(this);
    }

    public double getP(){
        return value[0];
    }

    public void setP(double kP){
        value[0] = kP;
    }

    public double getI(){
        return value[1];
    }

    public void setI(double kI){
        value[1] = kI;
    }

    public double getD(){
        return value[2];
    }

    public void setD(double kD){
        value[2] = kD;
    }

    public double getIZone(){
        return value[3];
    }

    public void setIZone(double kIZone){
        value[3] = kIZone;
    }

    public double getSetpoint(){
        return value[4];
    }

    public void setSetpoint(double setpoint){
        value[4] = setpoint;
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty(
            "izone",
            this::getIZone,
            (double toSet) -> {
            try {
                setIZone(toSet);
            } catch (IllegalArgumentException e) {
                MathSharedStore.reportError("IZone must be a non-negative number!", e.getStackTrace());
            }
            });
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }

    @Override
    public void periodic() {
        Logger.processInputs(prefix, inputs);
    }

}
