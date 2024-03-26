// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.Logger;

public class MAXSwerveModule {

  private final ModuleIO m_Io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;

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

  /**
   * Returns the current drive velocity of the module in feet per second.
   * 
   * @return The current velocity measurement of the module.
   */
  public double getVelocity() {
    return m_inputs.driveVelocity;
  }

  /**
   * Returns the current angular velocity of the module in rad per second.
   * 
   * @return The current angular velocity measurement of the module.
   */
  public double getTurnVelocity() {
    return m_inputs.turnVelocityRadPerSec;
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

  public boolean atDesiredAngle(){
    return Math.abs(m_desiredState.angle.getDegrees() - m_inputs.turnPosition.getDegrees()) <= 0.05;
  }

  /**
   * Optimizes the desired state for the module to avoid spinning further than 90 degrees.
   * 
   * @param desiredState
   */
  public SwerveModuleState getOptimizedState(SwerveModuleState desiredState) {
    return SwerveModuleState.optimize(desiredState, m_inputs.turnAbsolutePosition);  
  }

  public SwerveModuleState applyOffset(SwerveModuleState state){
    return m_Io.applyOffset(state);
  }

  public void setDriveVoltage(double volts){
    m_Io.setDriveVoltage(volts);
  }

  public void setTurnVoltage(double volts){
    m_Io.setTurnVoltage(volts);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetDriveEncoder() {
    m_Io.resetDriveEncoder();
  }

  public void setPID(){
    m_Io.setPID();
  }
}
