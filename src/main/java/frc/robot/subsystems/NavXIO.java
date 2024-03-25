// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C.Port;

/** Add your docs here. */
public class NavXIO implements GyroIO{
    private final AHRS navx = new AHRS(Port.kMXP);

    public NavXIO() {
        navx.setAngleAdjustment(180);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.heading = Rotation2d.fromDegrees(-navx.getAngle());
        inputs.yawVelocity = navx.getRate();
    }

    @Override
    public void reset() {
        navx.reset();
    }
}
