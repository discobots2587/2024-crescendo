// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.NavXIO;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.MAXModuleIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

// import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeConstants;

import frc.robot.commands.IntakeIndex;
import frc.robot.commands.ArmHold;

import com.pathplanner.lib.auto.AutoBuilder;

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

  //Auto chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_opController = new XboxController(OIConstants.kOpControllerPort);


  //Driver Buttons
  JoystickButton DriverIntakeBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
  JoystickButton SetXBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
  JoystickButton ZeroHeading = new JoystickButton(m_driverController, Button.kB.value);

  //Operator Buttons
  JoystickButton ArmIntakeMode = new JoystickButton(m_opController, Button.kA.value);
  JoystickButton ArmShootMode = new JoystickButton(m_opController, Button.kX.value);
  JoystickButton ArmAmpMode = new JoystickButton(m_opController, Button.kY.value);


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

    //Auto chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser()); // Default auto will be Commands.none()


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    intake.setDefaultCommand(new IntakeIndex(() -> DriverIntakeBumper.getAsBoolean()));
    arm.setDefaultCommand(new ArmHold(() -> ArmShootMode.getAsBoolean(), () -> ArmAmpMode.getAsBoolean()));
          
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
    SetXBumper.whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    ZeroHeading.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  }

  public void configureTestModeBindings(){
    new JoystickButton(m_driverController, Button.kY.value).onTrue(
                      new InstantCommand(() -> m_robotDrive.setPID(), m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // PathPlannerPath path = PathPlannerPath.fromPathFile("driveStraight");

    return autoChooser.get();
  }

  public static void getTeleopCommand() {

  }
}
