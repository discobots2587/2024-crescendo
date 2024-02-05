// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

    // m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    // m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // m_drivingSparkMax.restoreFactoryDefaults();
    // m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    // m_drivingEncoder = m_drivingSparkMax.getEncoder();
    // m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    // m_drivingPIDController = m_drivingSparkMax.getPIDController();
    // m_turningPIDController = m_turningSparkMax.getPIDController();
    // m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    // m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    // m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    // m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    // m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    // m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    // m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    // m_turningPIDController.setPositionPIDWrappingEnabled(true);
    // m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    // m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    // m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    // m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    // m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    // m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    // m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    // m_turningPIDController.setP(ModuleConstants.kTurningP);
    // m_turningPIDController.setI(ModuleConstants.kTurningI);
    // m_turningPIDController.setD(ModuleConstants.kTurningD);
    // m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    // m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    // m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    // m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    // m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    // m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

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
