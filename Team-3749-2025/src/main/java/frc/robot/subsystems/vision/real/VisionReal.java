package frc.robot.subsystems.vision.real;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.subsystems.vision.VisionIO;

public class VisionReal implements VisionIO {
    // * FROM VisionIO
    // PhotonPoseEstimator poseEstimatorList[] =
    // VisionConstants.CameraReal.poseEstimatorList;
    // PhotonCamera cameraList[] = VisionConstants.CameraReal.cameraList;

    private VisionData visionData;

    public VisionReal(VisionData visionData) {
        for (PhotonPoseEstimator poseEstimator : poseEstimatorList) {
            poseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
            poseEstimator.setFieldTags(VisionConstants.aprilTagFieldLayout);
        }

        this.visionData = visionData;
    }

    public void cameraUpdatePose(int index) {
        //! NOT updated to 2025, but still works

        PhotonCamera camera = cameraList[index];
        PhotonPoseEstimator poseEstimator = poseEstimatorList[index];

        //! marked for deprecation ... use these https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/using-target-data.html
        PhotonPipelineResult pipelineResult = camera.getLatestResult();
        
        double latencyMillis = pipelineResult.metadata.getLatencyMillis();
        visionData.latencyMillis[index] = latencyMillis;
        visionData.targetsSeen[index] = pipelineResult.getTargets().size();

        if (!pipelineResult.hasTargets())
            return;




        poseEstimator.setReferencePose(getReferencePose());
        var optional_robotPose = poseEstimator.update(pipelineResult);

        if (optional_robotPose.isEmpty())
            return;

        Pose2d robotPose = optional_robotPose.get().estimatedPose.toPose2d();
        visionData.visionEstimatedPose = robotPose;

        double timestamp = Timer.getFPGATimestamp() - latencyMillis / 1000.0;
        Robot.swerve.visionUpdateOdometry(robotPose, timestamp);
    }

    public void updatePose() {
        for (int i = 0; i < cameraList.length; i++) {
            cameraUpdatePose(i);
        }
    }
}
