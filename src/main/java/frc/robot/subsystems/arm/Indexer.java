package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase{

    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(int beamBreakChannel, int indexerID, int hoodID, double offset)
    {
        indexerIO = new SparkIndexerIO(beamBreakChannel, indexerID, hoodID, offset);
    }

    //Hood
    public Rotation2d getRawPosition()
    {
        return inputs.position; // returns in degrees
    }

    public Rotation2d getAiming()
    {
        return inputs.offsetPosition; // returns in degrees
    }

    public void setDesiredAngle(double desiredAngle)
    {
        indexerIO.setDesiredAngle(desiredAngle);
    }


    //Indexer rollers
    public void loadAndShoot()
    {
        indexerIO.setIndexerSpeed(IndexerConstants.kIntakeSpeed);
    }

    public void outtakeMode()
    {
        indexerIO.setIndexerSpeed(IndexerConstants.kOuttakeSpeed);
    }

    public void stowHood(){
        indexerIO.stowHood();
    }

    public void deployHood(){
        indexerIO.deployHood();
    }

    public void setIndexerSpeed(double speedRPM){
        indexerIO.setIndexerSpeed(speedRPM);
    }

    public void stop()
    {
        indexerIO.stop();
    }

    //Beambreak
    public boolean getBeamBreak()
    {
        return inputs.beambreak;
    }

    @Override
    public void periodic()
    {
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        if(!inputs.stowSwitch && inputs.hoodAppliedVolts <= -1e-6){
            indexerIO.stopHood();
        } else if(!inputs.deployedSwitch && inputs.hoodAppliedVolts >= 1e-6 ){
            indexerIO.stopHood();
        }
    }
}