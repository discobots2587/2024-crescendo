// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final Mode currentMode = Mode.REAL;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  //Field sim
  public final Field2d m_field = new Field2d();

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    //Drive base radius in meters. Distance from robot center to furthest module
    public static final double driveBaseRadius = 0.42207; //16.617 inches

    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 10;
    public static final int kFrontLeftDrivingCanId = 20;
    public static final int kRearLeftDrivingCanId = 30;
    public static final int kRearRightDrivingCanId = 40;

    public static final int kFrontRightTurningCanId = 11;
    public static final int kFrontLeftTurningCanId = 21;
    public static final int kRearLeftTurningCanId = 31;
    public static final int kRearRightTurningCanId = 41;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1; //1
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; //50 // amps
    public static final int kTurningMotorCurrentLimit = 20; //20 // amps
  }

  public static final class IntakeConstants
  {
    public static final int kIntakeCanID = 50; //TUNE


    public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;
    public static final int kIntakeMotorCurrentLimit = 50; //50 amps
    
    public static final double kIntakeSpeed = 1; // TUNE
    public static final double kOuttakeSpeed = -1; // TUNE
  }

  public static final class PivotConstants
  {
    public static final int kMasterID = 70; //TUNE
    // public static final int kSlaveID = 99; //TUNE

    public static final IdleMode kPivotIdleMode = IdleMode.kBrake;
    public static final int kMotorCurrentLimit = 20;//20 amps TUNE
    
    public static final double kTurningEncoderPositionFactor = (360); // degrees
    public static final double kTurningEncoderVelocityFactor = (360) / 60.0; // degrees per second


    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;
    
  }

  public static final class FlywheelConstants
  {
    public static final int kMasterID = 71; //TUNE
    public static final int kSlaveID = 72; //TUNE

    public static final IdleMode kFlywheelIdleMode = IdleMode.kCoast;
    public static final int kMotorCurrentLimit = 20;//20 amps TUNE
    
    public static final double kTurningEncoderVelocityFactor = 1/60.0; // degrees per second


    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
    public static final double kVelocityMinOutput = -1;
    public static final double kVelocityMaxOutput = 1;
  }

  public static final class IndexerConstants
  {
    public static final int kIndexerID = 60; //TUNE
    public static final int kHoodID = 73; //TUNE

    public static final int kBeamBreakID = 0; 

    public static final IdleMode kIndexerIdleMode = IdleMode.kCoast;
    public static final IdleMode kHoodIdleMode = IdleMode.kBrake;

    public static final int kHoodCurrentLimit = 20; //20 amps TUNE
    public static final int kIndexerCurrentLimit = 20; //20 amps TUNE

    public static final double kOffset = 60; // TUNE 
    
    public static final double kTurningEncoderPositionFactor = (360); // degrees
    public static final double kTurningEncoderVelocityFactor = 1/60.0; // degrees per second

    //Hood PID
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
    public static final double kVelocityMinOutput = -1;
    public static final double kVelocityMaxOutput = 1;

    //Speed constant
    public static double kIntakeSpeed = 1;
    public static double kOuttakeSpeed = -1;
    // public static double kBeamBreakThresh = 0.75;
  }

  public static final class ArmConstants
  {
    public enum ArmState
    {
      INTAKE,
      SHOOTER,
      AMP
    }

    //All of these constants are in degrees
    public static double kPivotOffset = 60; //TUNE 
    public static double kHoodOffset = 60; //TUNE

    public static double PivotIntakePosition = 53.7; //TUNE
    public static double PivotAmpPosition = 180; //TUNE

    public static double HoodAmpPosition = 147; //TUNE
    public static double HoodStowPosition = 0; //TUNE
  }

  public static final class VisionConstants {

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;

    public static final int kOpControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3; //3
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; //3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.4; //1
    public static final double kPYController = 0.4; //1
    public static final double kPThetaController = 0.4; //1

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
