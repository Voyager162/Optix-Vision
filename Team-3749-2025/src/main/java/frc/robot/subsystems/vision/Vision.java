package frc.robot.subsystems.vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.*;
import frc.robot.subsystems.vision.real.Photonvision;
import frc.robot.subsystems.vision.sim.VisionSim;

public class Vision extends SubsystemBase {
    @SuppressWarnings("unused")
    private VisionIO visionIO;
    private VisionData visionData = new VisionData();

    public Vision() {
        if (Robot.isReal()) {
            visionIO = new Photonvision(visionData);
        } else {
            visionIO = new VisionSim();
        }

    }

    @Override
    public void periodic() {
        visionIO.updatePose();
    }

    /**
     * 
     * @param strat what multi-tag fallback strategy to use
     * @param index what camera to change the strategy of (0-5)
     */
    public void setCameraStrategy(PoseStrategy strat, int index) {
        visionIO.setCameraStrategy(strat, index);
    }


}
