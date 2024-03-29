// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.NavXIO;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.MAXModuleIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmHold;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DriveMotorFFCharacterization;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.IntakeIndex;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  public static final Intake intake = new Intake(IntakeConstants.kIntakeCanID);
  public static final Arm arm = new Arm();
  public static final Climber climber = new Climber(ClimberConstants.kLeftClimberCanID, ClimberConstants.kRightClimberCanID, ClimberConstants.kLeftSwitchPort, ClimberConstants.kRightSwitchPort);

  private static double scaler = 1.0; // Scales the velocity of the robot

  //Auto chooser
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> feedForwardChooser;

  // Controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_opController = new XboxController(OIConstants.kOpControllerPort);

  //Driver Buttons
  JoystickButton DriverIntakeBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
  JoystickButton DriverOuttake = new JoystickButton(m_driverController, Button.kRightBumper.value);

  JoystickButton ZeroHeading = new JoystickButton(m_driverController, Button.kB.value);
  JoystickButton RobotCentric = new JoystickButton(m_driverController, Button.kA.value);
  JoystickButton FeedForwardTest = new JoystickButton(m_driverController, Button.kX.value);
  JoystickButton SlowMode = new JoystickButton(m_driverController, Button.kRightStick.value);

  DoubleSupplier leftOpSup = () -> m_opController.getLeftY();
  DoubleSupplier rightOpSup = () -> m_opController.getRightY();

  //Operator Buttons
  JoystickButton ArmIntakeMode = new JoystickButton(m_opController, Button.kA.value);
  JoystickButton ArmShootMode = new JoystickButton(m_opController, Button.kX.value);
  JoystickButton ArmAmpMode = new JoystickButton(m_opController, Button.kY.value);
  JoystickButton TestShooter = new JoystickButton(m_opController, Button.kRightBumper.value);

  JoystickButton ClimbDeploy = new JoystickButton(m_opController, Button.kLeftBumper.value);
  JoystickButton ClimbReverse = new JoystickButton(m_opController, Button.kB.value);

  // JoystickButton RightClimbDown = new JoystickButton(m_opController, Button.kRightBumper.value);
  // JoystickButton LeftClimbDown = new JoystickButton(m_opController, Button.kLeftBumper.value);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_robotDrive = new DriveSubsystem(new NavXIO(),
                                      new MAXModuleIO(0), //FrontLeft
                                      new MAXModuleIO(1), //FrontRight
                                      new MAXModuleIO(2), //RearLeft
                                      new MAXModuleIO(3)); //RearRight

    // Configure the button bindings
    configureButtonBindings();

    //Register named commands for auto
    NamedCommands.registerCommand("Shoot", new AutoShoot());
    NamedCommands.registerCommand("Intake", new AutoIntake());
    
    //Auto chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser()); // Default auto will be Commands.none()
    feedForwardChooser = new LoggedDashboardChooser<>("System ID Routine Chooser");

    //System Identification setup
    feedForwardChooser.addOption("Drive FF Characterization", new SequentialCommandGroup(new InstantCommand(() -> m_robotDrive.setFFAngles()), 
                                                                                              new FeedForwardCharacterization(m_robotDrive, 
                                                                                                  m_robotDrive::runDriveCharacterizationVolts, 
                                                                                                  () -> m_robotDrive.getModuleVelocity(0),
                                                                                                  () -> m_robotDrive.getModuleVelocity(1),
                                                                                                  () -> m_robotDrive.getModuleVelocity(2),
                                                                                                  () -> m_robotDrive.getModuleVelocity(3))));
    feedForwardChooser.addOption("Drive Forward Quasistatic SysId", m_robotDrive.getDriveQuasistaticSysId(Direction.kForward)
                                                                              .beforeStarting(new SequentialCommandGroup(
                                                                                          new InstantCommand(() -> m_robotDrive.setFFAngles()),
                                                                                          new WaitUntilCommand(() -> m_robotDrive.atDesiredAngle()))));
    feedForwardChooser.addOption("Drive Backwards Quasistatic SysId", m_robotDrive.getDriveQuasistaticSysId(Direction.kReverse)
                                                                              .beforeStarting(new SequentialCommandGroup(
                                                                                          new InstantCommand(() -> m_robotDrive.setFFAngles()),
                                                                                          new WaitUntilCommand(() -> m_robotDrive.atDesiredAngle()))));
    feedForwardChooser.addOption("Drive Forward Dynamic SysId", m_robotDrive.getDriveDynamicSysId(Direction.kForward)
                                                                              .beforeStarting(new SequentialCommandGroup(
                                                                                          new InstantCommand(() -> m_robotDrive.setFFAngles()),
                                                                                          new WaitUntilCommand(() -> m_robotDrive.atDesiredAngle()))));
    feedForwardChooser.addOption("Drive Backwards Dynamic SysId", m_robotDrive.getDriveDynamicSysId(Direction.kReverse)
                                                                              .beforeStarting(new SequentialCommandGroup(
                                                                                          new InstantCommand(() -> m_robotDrive.setFFAngles()),
                                                                                          new WaitUntilCommand(() -> m_robotDrive.atDesiredAngle()))));


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)*scaler,
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)*scaler,
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)*scaler,
                !RobotCentric.getAsBoolean(), true),//Added the robot centric button
            m_robotDrive));

    // intake.setDefaultCommand(new IntakeIndex(() -> DriverIntakeBumper.getAsBoolean()));
    // intake.setDefaultCommand(new IntakeTest(leftOpSup, rightOpSup));

    // arm.setDefaultCommand(new ArmHold(() -> ArmShootMode.getAsBoolean(), () -> ArmAmpMode.getAsBoolean()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings()
  {
    DriverOuttake.onTrue(new InstantCommand(() -> {
      intake.outtake();
      arm.ampOuttake();
    }, intake));
    DriverOuttake.onFalse(new InstantCommand(() -> {
      intake.stop();
      arm.indexStop();
    }, intake));
    DriverIntakeBumper.whileTrue(new IntakeIndex(true));
    TestShooter.onTrue(new InstantCommand(() -> arm.setFlywheelVoltage(9.5), arm));
    TestShooter.onFalse(new InstantCommand(() -> arm.stopFlywheel(), arm));

    SlowMode.onTrue(new InstantCommand(() -> scaler = (scaler == 1.0)? 0.5 : 1.0));

    ZeroHeading.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    ArmShootMode.onTrue(new InstantCommand(() -> arm.shooterMode(10), arm)); // Desired angle needs to use auto aim command
    ArmAmpMode.whileTrue(new InstantCommand(() -> arm.ampMode(), arm));

    ArmShootMode.or(ArmAmpMode).onFalse(new InstantCommand(() -> arm.intakeMode(), arm));
    
    ClimbDeploy.onTrue(new InstantCommand(() -> climber.spinBoth()));
    ClimbDeploy.onFalse(new InstantCommand(() -> climber.stopBoth()));

    ClimbReverse.onTrue(new InstantCommand(() -> climber.reverseBoth()));
    ClimbReverse.onFalse(new InstantCommand(() -> climber.stopBoth()));

    // RightClimbDown.whileTrue(new InstantCommand(() -> climber.setRightDesiredPosition(0)));
    // RightClimbDown.onFalse(new InstantCommand(() -> climber.stopRight()));

    // LeftClimbDown.whileTrue(new InstantCommand(() -> climber.setLeftDesiredPosition(0)));
    // LeftClimbDown.onFalse(new InstantCommand(() -> climber.stopLeft()));
  }
  public void configureTestModeBindings(){
    FeedForwardTest.whileTrue(feedForwardChooser.get());
    new JoystickButton(m_driverController, Button.kY.value).onTrue(
                      new InstantCommand(() -> m_robotDrive.setPID(), m_robotDrive));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_robotDrive.zeroHeading();
    return autoChooser.get();
  }

  public static void getTeleopCommand() {

  }
}