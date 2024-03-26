package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.utils.LoggedDashboardPID;

public class MAXModuleIO implements ModuleIO{
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;
  private final int id;

  private final LoggedDashboardPID drivekPID, turnkPID;
  private final LoggedDashboardNumber /*driveKP, driveKI, driveKD,*/ driveKFF, driveArbFF;
  private final LoggedDashboardNumber /*turnKP, turnKI, turnKD,*/ turnKFF, turnArbFF;

  private double m_drivingArbFF = 0.0;
  private double m_turningArbFF = 0.0;
  private double m_chassisAngularOffset = 0.0;

    public MAXModuleIO(int id){
        this.id = id;
        switch(id){
            case 0: //FrontLeft
                m_drivingSparkMax = new CANSparkMax(DriveConstants.kFrontLeftDrivingCanId, MotorType.kBrushless);
                m_turningSparkMax = new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
                m_chassisAngularOffset = DriveConstants.kFrontLeftChassisAngularOffset;
                break;

            case 1: //FrontRight
                m_drivingSparkMax = new CANSparkMax(DriveConstants.kFrontRightDrivingCanId, MotorType.kBrushless);
                m_turningSparkMax = new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless);
                m_chassisAngularOffset = DriveConstants.kFrontRightChassisAngularOffset;                
                break;

            case 2: //RearLeft
                m_drivingSparkMax = new CANSparkMax(DriveConstants.kRearLeftDrivingCanId, MotorType.kBrushless);
                m_turningSparkMax = new CANSparkMax(DriveConstants.kRearLeftTurningCanId, MotorType.kBrushless);
                m_chassisAngularOffset = DriveConstants.kBackLeftChassisAngularOffset;            
                break;

            case 3: //RearRight
                m_drivingSparkMax = new CANSparkMax(DriveConstants.kRearRightDrivingCanId, MotorType.kBrushless);
                m_turningSparkMax = new CANSparkMax(DriveConstants.kRearRightTurningCanId, MotorType.kBrushless);
                m_chassisAngularOffset = DriveConstants.kBackRightChassisAngularOffset;
                break;

            default:
                throw new RuntimeException("Invalid Module Index");
        }

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();

        //Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_turningPIDController = m_turningSparkMax.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        //Enable PID wrap around for the turning motor. This will allow the PID
        //controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        //to 10 degrees will go through 0 rather than the other direction which is a
        //longer route.
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_drivingPIDController.setP(ModuleConstants.kDrivingP);
        m_drivingPIDController.setI(ModuleConstants.kDrivingI);
        m_drivingPIDController.setD(ModuleConstants.kDrivingD);
        m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_turningPIDController.setP(ModuleConstants.kTurningP);
        m_turningPIDController.setI(ModuleConstants.kTurningI);
        m_turningPIDController.setD(ModuleConstants.kTurningD);
        m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

        m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();

        m_drivingEncoder.setPosition(0);


        // driveKP = new LoggedDashboardNumber("driveKP[" + id + "]", ModuleConstants.kDrivingP);
        // driveKI = new LoggedDashboardNumber("driveKI[" + id + "]", ModuleConstants.kDrivingI);
        // driveKD = new LoggedDashboardNumber("driveKP[" + id + "]", ModuleConstants.kDrivingD);
        drivekPID = new LoggedDashboardPID("drivekPID[" + id + "]", new double[]{ModuleConstants.kDrivingP,
                                                                                ModuleConstants.kDrivingI,
                                                                                ModuleConstants.kDrivingD,
                                                                                Double.POSITIVE_INFINITY,
                                                                                0.0});
        driveKFF = new LoggedDashboardNumber("driveKFF[" + id + "]", ModuleConstants.kDrivingFF);
        driveArbFF = new LoggedDashboardNumber("driveArbFF[" + id + "]", 0);
        
        // turnKP = new LoggedDashboardNumber("turnKP[" + id + "]", ModuleConstants.kTurningP);
        // turnKI = new LoggedDashboardNumber("turnKI[" + id + "]", ModuleConstants.kTurningI);
        // turnKD = new LoggedDashboardNumber("turnKD[" + id + "]", ModuleConstants.kTurningD);
        turnkPID = new LoggedDashboardPID("drivekPID[" + id + "]", new double[]{ModuleConstants.kTurningP,
                                                                                ModuleConstants.kTurningI,
                                                                                ModuleConstants.kTurningD,
                                                                                Double.POSITIVE_INFINITY,
                                                                                0.0});
        turnKFF = new LoggedDashboardNumber("turnKFF[" + id + "]", ModuleConstants.kTurningFF);
        turnArbFF = new LoggedDashboardNumber("turnArbFF[" + id + "]", 0);

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePosition = m_drivingEncoder.getPosition();
        inputs.driveVelocity = m_drivingEncoder.getVelocity();
        inputs.driveAppliedVolts = m_drivingSparkMax.getAppliedOutput() * m_drivingSparkMax.getBusVoltage();
        inputs.driveCurrentAmps = new double[] {m_drivingSparkMax.getOutputCurrent()};

        inputs.turnAbsolutePosition = Rotation2d.fromRadians(m_turningEncoder.getPosition() - m_chassisAngularOffset); //Chassis Relative
        inputs.turnPosition = Rotation2d.fromRadians(m_turningEncoder.getPosition());
        inputs.turnVelocityRadPerSec = m_turningEncoder.getVelocity();
        inputs.turnAppliedVolts = m_turningSparkMax.getAppliedOutput() * m_turningSparkMax.getBusVoltage();
        inputs.turnCurrentAmps = new double[] {m_turningSparkMax.getOutputCurrent()};
    }

    @Override
    public void setReference(SwerveModuleState reference){
        // Change reference (desired state) to be field relative
        reference.angle = reference.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        m_drivingPIDController.setReference(reference.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, m_drivingArbFF, ArbFFUnits.kVoltage);
        m_turningPIDController.setReference(reference.angle.getRadians(), CANSparkMax.ControlType.kPosition, 0, m_turningArbFF);
    }

    @Override
    public void setDriveVoltage(double volts) {
        m_drivingSparkMax.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        m_turningSparkMax.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        m_drivingSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        m_turningSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void resetDriveEncoder(){
        m_drivingEncoder.setPosition(0);
    }

    @Override
    public SwerveModuleState applyOffset(SwerveModuleState state){
        state.angle = state.angle.minus(Rotation2d.fromRadians(m_chassisAngularOffset));
        return state;
    }

    @Override
    public void setPID(){
        // m_drivingPIDController.setP(driveKP.get());
        // m_drivingPIDController.setI(driveKI.get());
        // m_drivingPIDController.setD(driveKD.get());
        m_drivingPIDController.setP(drivekPID.getP());
        m_drivingPIDController.setI(drivekPID.getI());
        m_drivingPIDController.setD(drivekPID.getD());
        m_drivingPIDController.setFF(driveKFF.get());
        m_drivingArbFF = driveArbFF.get();

        // m_turningPIDController.setP(turnKP.get());
        // m_turningPIDController.setI(turnKI.get());
        // m_turningPIDController.setD(turnKD.get());
        m_turningPIDController.setP(turnkPID.getP());
        m_turningPIDController.setI(turnkPID.getI());
        m_turningPIDController.setD(turnkPID.getD());
        m_turningPIDController.setFF(turnKFF.get());
        m_turningArbFF = turnArbFF.get();
    }

}
