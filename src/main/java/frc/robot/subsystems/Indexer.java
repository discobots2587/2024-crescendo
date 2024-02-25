package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase{
    private final CANSparkMax HoodSpark;
    private final CANSparkMax IndexerSpark;


    private final AbsoluteEncoder hoodEnc;
    private final SparkPIDController hoodPID;

    private final DigitalInput beambreakDigitalInput;

    private final double angleOffset;

    public Indexer(int beamBreakChannel, int indexerID, int hoodID, double offset)
    {
        IndexerSpark = new CANSparkMax(indexerID, MotorType.kBrushless);
        HoodSpark = new CANSparkMax(hoodID, MotorType.kBrushless);

        angleOffset = offset;

        beambreakDigitalInput = new DigitalInput(beamBreakChannel);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        IndexerSpark.restoreFactoryDefaults();
        HoodSpark.restoreFactoryDefaults();

        // Setup encoders and PID controller for the pivot sparkmax.
        hoodEnc = HoodSpark.getAbsoluteEncoder(Type.kDutyCycle);
        hoodPID = HoodSpark.getPIDController();
        hoodPID.setFeedbackDevice(hoodEnc);
        // hoodPID.

        // Apply position and velocity conversion factors for the turning encoder.
        hoodEnc.setVelocityConversionFactor(IndexerConstants.kTurningEncoderVelocityFactor);

        
        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        hoodPID.setP(IndexerConstants.kP);
        hoodPID.setI(IndexerConstants.kI);
        hoodPID.setD(IndexerConstants.kD);
        hoodPID.setFF(IndexerConstants.kFF);
        hoodPID.setOutputRange(IndexerConstants.kVelocityMinOutput, IndexerConstants.kVelocityMaxOutput);
        
        HoodSpark.setIdleMode(IndexerConstants.kHoodIdleMode);
        IndexerSpark.setIdleMode(IndexerConstants.kIndexerIdleMode);

        HoodSpark.setSmartCurrentLimit(IndexerConstants.kHoodCurrentLimit);
        IndexerSpark.setSmartCurrentLimit(IndexerConstants.kIndexerCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        HoodSpark.burnFlash();
        IndexerSpark.burnFlash();
    }


    //Hood
    public double getRawPosition()
    {
        return hoodEnc.getPosition(); // returns in degrees
    }

    public double getAiming()
    {
        return getRawPosition() - angleOffset; // returms in degrees
    }

    public void setDesiredAngle(double desiredAngle)
    {
        double turnPose = desiredAngle - angleOffset;

        SmartDashboard.putNumber("Target Position", turnPose); 
        hoodPID.setReference(turnPose, CANSparkMax.ControlType.kPosition);
    }


    //Indexer rollers
    public void loadAndShoot()
    {
        IndexerSpark.set(IndexerConstants.kIntakeSpeed);
    }

    public void outtakeMode()
    {
        IndexerSpark.set(IndexerConstants.kOuttakeSpeed);
    }

    public void stop()
    {
        IndexerSpark.stopMotor();
    }

    //Beambreak
    public boolean getBeamBreak()
    {
        if (beambreakDigitalInput.get()){return true;}
        else{return false;}
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Aiming Position", getAiming()); // log the aiming position of the arm.
    }
}