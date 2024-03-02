package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;


public class Arm extends SubsystemBase
{
    private final Pivot pivot;// = new Pivot(PivotConstants.kMasterID, PivotConstants.kOffset);
    private final Indexer indexer;// = new Indexer(IndexerConstants.kIndexerID, IndexerConstants.kHoodID, IndexerConstants.kOffset);
    private final Shooter flywheelShooter;// = new Shooter(FlywheelConstants.kMasterID, FlywheelConstants.kSlaveID);

    private ArmConstants.ArmState state;
    
    public Arm()
    {
        pivot = new Pivot(PivotConstants.kMasterID, ArmConstants.kPivotOffset);
        indexer = new Indexer(IndexerConstants.kBeamBreakID, IndexerConstants.kIndexerID, IndexerConstants.kHoodID, ArmConstants.kHoodOffset);
        flywheelShooter = new Shooter(FlywheelConstants.kMasterID, FlywheelConstants.kSlaveID);
        
        state = ArmConstants.ArmState.INTAKE;
    }

    public ArmConstants.ArmState getState()
    {
        return state;
    }

    public void intakeMode()
    {
        state = ArmConstants.ArmState.INTAKE;
        pivot.setDesiredAngle(ArmConstants.PivotIntakePosition);
        indexer.setDesiredAngle(ArmConstants.HoodStowPosition);
    }

    public void shooterMode(double desiredAngle)//This allows for the arm to track 
    {
        state = ArmConstants.ArmState.SHOOTER;
        pivot.setDesiredAngle(desiredAngle);
        indexer.setDesiredAngle(ArmConstants.HoodStowPosition);
    }

    public void ampMode()
    {
        state = ArmConstants.ArmState.AMP;
        pivot.setDesiredAngle(ArmConstants.PivotAmpPosition);
        indexer.setDesiredAngle(ArmConstants.HoodAmpPosition);
    }

    //Flywheel actions
    public void setFlywheelVelocity(double velocity)
    {
        flywheelShooter.setDesiredVelocity(velocity);
    }

    public void setFlywheelVoltage(double voltage)
    {
        flywheelShooter.setDesiredVelocity(voltage);
    }

    //Roller actions
    public void shoot()
    {
        indexer.loadAndShoot();
    }

    public void ampOuttake()
    {
        indexer.outtakeMode();
    }

    public void load()
    {
        if(indexer.getBeamBreak())
        {indexer.loadAndShoot();}
        else
        {indexer.stop();}
    }

    public void indexStop()
    {
        indexer.stop();
    }

    public void indexTest(double speed)
    {
        indexer.setSpeed(speed);
    }

    public void shooterTest(double velocity)
    {
        flywheelShooter.setDesiredVelocity(velocity);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putString("Arm State", state.toString());
    }

    public void stopFlywheel() {
        flywheelShooter.stop();
    }
}

