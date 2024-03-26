// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.WPIUtilJNI;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.GyroIOInputsAutoLogged;
import frc.utils.LocalADStarAK;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft;
  private final MAXSwerveModule m_frontRight;
  private final MAXSwerveModule m_rearLeft;
  private final MAXSwerveModule m_rearRight;

  // The gyro sensor
  private final GyroIO gyroIo;
  private final GyroIOInputsAutoLogged g_inputs = new GyroIOInputsAutoLogged();

  // Field
  private Field2d field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private SysIdRoutine driveSysId = new SysIdRoutine(
    new SysIdRoutine.Config(
      null, null, null,
      (state) -> Logger.recordOutput("Drive/SysIdTestState", state.toString())
    ),
    new SysIdRoutine.Mechanism(
      (voltage) -> this.runDriveCharacterizationVolts(voltage.in(Units.Volts)),
      null,
      this
    )
  );

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(GyroIO gyroIo , ModuleIO frontLeftIo, ModuleIO frontRightIo, ModuleIO rearLeftIo, ModuleIO rearRightIo) {

    //Configure Modules
    m_frontLeft = new MAXSwerveModule(0, frontLeftIo);
    m_frontRight = new MAXSwerveModule(1, frontRightIo);
    m_rearLeft = new MAXSwerveModule(2, rearLeftIo);
    m_rearRight = new MAXSwerveModule(3, rearRightIo);

    this.gyroIo = gyroIo;

    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      g_inputs.heading,
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

    //Pathplanner configuration
    AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                        AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                        DriveConstants.driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red; //Red (blue??)
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        Pathfinding.setPathfinder(new LocalADStarAK());

        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
              Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            }
        );
        PathPlannerLogging.setLogTargetPoseCallback(
          (targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetPoint", targetPose);
          }
        );
        SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    gyroIo.updateInputs(g_inputs);
    Logger.processInputs("Drive/Gyro", g_inputs);

    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();

    // Update the odometry in the periodic block
    m_odometry.update(
        g_inputs.heading,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    
    field.setRobotPose(getPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    gyroIo.reset();
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(180),// new Rotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, g_inputs.heading)
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    setModuleStates(new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    });
  }

  /**
   * Sets the wheels with bevels facing away from center. Used for drive motor feedforward routine.
   */
  public void setFFAngles(){
    setModuleStates(new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45 + 180)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45 + 180))},
          false
    );
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    Logger.recordOutput("SwerveStates/SetPoints", offsetAngles(desiredStates));
    desiredStates[0] = m_frontLeft.getOptimizedState(desiredStates[0]);
    desiredStates[1] = m_frontLeft.getOptimizedState(desiredStates[1]);
    desiredStates[2] = m_frontLeft.getOptimizedState(desiredStates[2]);
    desiredStates[3] = m_frontLeft.getOptimizedState(desiredStates[3]);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
    Logger.recordOutput("SwerveStates/OptimizedSetPoints", offsetAngles(desiredStates));
  }

  /**
   * Sets the swerve ModuleStates. For driving, it should almost always be optimized.
   * This method is only for testing or system identification.
   * 
   * @param desiredStates The desired SwerveModule states.
   * @param optimize Whether the states should be optimized (desaturate wheelspeeds and minimize change in angle)
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean optimize){
    if(optimize){
      setModuleStates(desiredStates);
    } else {
      Logger.recordOutput("SwerveStates/SetPoints", desiredStates);
      Logger.recordOutput("SwerveStates/OptimizedSetPoints", desiredStates);
      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_rearLeft.setDesiredState(desiredStates[2]);
      m_rearRight.setDesiredState(desiredStates[3]);
    }
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();

    return states;
  }


  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetDriveEncoder();
    m_rearLeft.resetDriveEncoder();
    m_frontRight.resetDriveEncoder();
    m_rearRight.resetDriveEncoder();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyroIo.reset();
  }

  public void runDriveCharacterizationVolts(double volts){
    m_frontLeft.setDriveVoltage(volts);
    m_rearLeft.setDriveVoltage(volts);
    m_frontRight.setDriveVoltage(volts);
    m_rearRight.setDriveVoltage(volts);
  }

  /**
   * UNTESTED FUNCTIONALITY
   * Checks if a module are at their set desired state
   * 
   * 
   * @param id Id value of module to check
   */
  public boolean atDesiredAngle(int id){
    switch(id){
      case 0:
        return m_frontLeft.atDesiredAngle();
      case 1:
        return m_rearLeft.atDesiredAngle();
      case 2:
        return m_frontRight.atDesiredAngle();
      case 3:
        return m_rearRight.atDesiredAngle();
      default:
        return false;
    }
  }

  @AutoLogOutput(key = "SwerveStates/atDesired")
  public boolean atDesiredAngle(){
    return atDesiredAngle(0) && atDesiredAngle(1) && atDesiredAngle(2) && atDesiredAngle(3);
  }

  public double getModuleVelocity(int id){
    switch(id){
      case 0:
        return m_frontLeft.getVelocity();
      case 1:
        return m_rearLeft.getVelocity();
      case 2:
        return m_frontRight.getVelocity();
      case 3:
        return m_rearRight.getVelocity();
      default:
        throw new Error("Invalid Module Index");
    }
  }

  public SwerveModuleState[] offsetAngles(SwerveModuleState[] states){
    return new SwerveModuleState[]{
      m_frontLeft.applyOffset(states[0]),
      m_rearLeft.applyOffset(states[1]),
      m_frontRight.applyOffset(states[2]),
      m_rearRight.applyOffset(states[3])
    };
  }

  public Command getDriveQuasistaticSysId(Direction direction){
    return driveSysId.quasistatic(direction);
  }

  public Command getDriveDynamicSysId(Direction direction){
    return driveSysId.dynamic(direction);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return g_inputs.heading.getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return g_inputs.yawVelocity * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /* (for Test Mode) Sets drive PID values using values on SmartDashboard */
  public void setPID(){
    m_frontLeft.setPID();
    m_frontRight.setPID();
    m_rearLeft.setPID();
    m_rearRight.setPID();
  }
}
