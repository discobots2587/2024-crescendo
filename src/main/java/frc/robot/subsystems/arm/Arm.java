package frc.robot.subsystems.arm;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private static ArmConstants.ArmState state;
    
    public Arm()
    {
        pivot = new Pivot(PivotConstants.kPivotCanID, ArmConstants.kPivotOffset);
        indexer = new Indexer(IndexerConstants.kBeamBreakID, IndexerConstants.kIndexerCanID, IndexerConstants.kHoodCanID, ArmConstants.kHoodOffset);
        flywheelShooter = new Shooter(FlywheelConstants.kFlywheelMasterCanID, FlywheelConstants.kFlywheelSlaveCanID);
        
        state = ArmConstants.ArmState.INTAKE;
    }

    public Pivot getPivot(){
        return pivot;
    }
    public Indexer getIndexer(){
        return indexer;
    }
    public Shooter getShooter(){
        return flywheelShooter;
    }

    public ArmConstants.ArmState getState()
    {
        return state;
    }

    public void intakeMode()
    {
        state = ArmConstants.ArmState.INTAKE;
        pivot.setDesiredAngle(ArmConstants.PivotIntakePosition);
        // indexer.setDesiredAngleinde.(ArmConstants.HoodStowPosition);
        indexer.stowHood();
    }

    public void shooterMode(double desiredAngle) //This allows for the arm to track 
    {
        state = ArmConstants.ArmState.SHOOTER;
        pivot.setDesiredAngle(desiredAngle);
        // indexer.setDesiredAngle(ArmConstants.HoodStowPosition);
        indexer.stowHood();
    }

    public void shooterMode(Pose2d robotPose) //This allows for the arm to track 
    {
        state = ArmConstants.ArmState.SHOOTER;
        pivot.setDesiredAngle(0); // Make this do the math for the vision thingie and the angle it needs to shoot at.
        // indexer.setDesiredAngle(ArmConstants.HoodStowPosition);
        indexer.stowHood();
    }

    public void ampMode()
    {
        state = ArmConstants.ArmState.AMP;
        pivot.setDesiredAngle(ArmConstants.PivotAmpPosition);
        // indexer.setDesiredAngle(ArmConstants.HoodAmpPosition);
        indexer.deployHood();
    }

    //Flywheel actions

    /**
     * Set desired flywheel velocity in degrees per second.
     * @param velocity Angular velocity in deg per second
     */
    public void setFlywheelVelocity(double velocity)
    {
        flywheelShooter.setDesiredVelocity(velocity);
    }

    public void setFlywheelVoltage(double voltage)
    {
        flywheelShooter.setDesiredVoltage(voltage);
    }

    /**
     * Get the measured velocity of the flywheel in degrees per second.
     * @return measured angular velocity
     */
    public double getFlywheelVelocity(){
        return flywheelShooter.getVelocity();
    }

    public void stopFlywheel(){
        flywheelShooter.stop();
    }

    // pivot actions

    public void setPivotVoltage(double volts){
        pivot.setVoltage(volts);
    }

    public double getPivotVelocity(){
        return pivot.getVelocity();
    }

    //Roller actions
    /**
     * Runs rollers on indexer to intake into shooter.
     */
    public void shoot()
    {
        indexer.loadAndShoot();
    }

    /**
     * Runs rollers on indexer to outtake into amp.
     */
    public void ampOuttake()
    {
        indexer.outtakeMode();
    }

    public void stopIndexer(){
        indexer.stop();
    }

    /**
     * Runs rollers on indexer to load note and stops when note is detected by beambreak.
     */
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

    @Override
    public void periodic()
    {
        Logger.recordOutput("Arm/State", state.toString());
    }
}

