// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    // private final double angleOffset; // This is the initial value of the arm. This shoukld be aiming upwards at about 60 degrees.

    public Pivot(int sparkID, double offset)
    {
        pivotIO = new SparkPivotIO(sparkID, offset);
    }

    /* 
     * Get the position given by the absolute encoder
     * 
     * @return Rotation2d of angle
    */
    public Rotation2d getPosition()
    {
        return pivotInputs.position; // returns in degrees
    }

    /*
     * Get the angle of the pivot from the offset angle
     * 
     * @return Rotation2d of angle
     */
    public Rotation2d getAiming()
    {
        return pivotInputs.offsetPosition; // returns in degrees
    }

    public void setDesiredAngle(double desiredAngle)
    {
        pivotIO.setDesiredAngle(desiredAngle);
    }

    public void setDesiredAngle(Rotation2d desiredAngle){
        pivotIO.setDesiredAngle(desiredAngle.getDegrees());
    }

    @Override
    public void periodic()
    {
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("Arm/Pivot", pivotInputs);
    }
}
