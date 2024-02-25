// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import frc.robot.subsystems.ModuleIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class MAXSwerveModule {

  private final ModuleIO m_Io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;

  // private final CANSparkMax m_drivingSparkMax;
  // private final CANSparkMax m_turningSparkMax;

  // private final RelativeEncoder m_drivingEncoder;
  // private final AbsoluteEncoder m_turningEncoder;

  // private final SparkPIDController m_drivingPIDController;
  // private final SparkPIDController m_turningPIDController;

  // private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int id, ModuleIO io) {
    m_Io = io;
    m_index = id;

    m_desiredState.angle = m_inputs.turnPosition;
    resetDriveEncoder();
  }

  public void periodic() {
    m_Io.updateInputs(m_inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(m_index), m_inputs);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState( m_inputs.driveVelocity, m_inputs.turnAbsolutePosition );
  }
  
  // public void runCharacterization(double volts) {
  //   m_desiredState = null;
  // }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition( m_inputs.drivePosition, m_inputs.turnAbsolutePosition );
  }

  /**
   * Sets the desired state (chassis relative) for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_Io.setReference(desiredState);

    m_desiredState = desiredState;
  }

  /**
   * Optimizes the desired state for the module to avoid spinning further than 90 degrees.
   * 
   * @param desiredState
   */
  public SwerveModuleState getOptimizedState(SwerveModuleState desiredState) {
    return SwerveModuleState.optimize(desiredState, m_inputs.turnAbsolutePosition);  
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetDriveEncoder() {
    m_Io.resetDriveEncoder();
  }
}