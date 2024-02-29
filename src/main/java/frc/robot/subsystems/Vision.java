package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase
{
    private PIDController forwardController = new PIDController(VisionConstants.LINEAR_P, 0, VisionConstants.LINEAR_D);
    private PIDController turnController = new PIDController(VisionConstants.ANGULAR_P, 0, VisionConstants.ANGULAR_D);

    PhotonCamera camera = new PhotonCamera("photonvision");

    
}
