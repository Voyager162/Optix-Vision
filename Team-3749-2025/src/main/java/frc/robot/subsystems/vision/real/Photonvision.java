package frc.robot.subsystems.vision.real;

import java.util.List;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;

public class Photonvision implements VisionIO {
    // * FROM VisionIO
    // PhotonPoseEstimator poseEstimatorList[] =
    // VisionConstants.CameraReal.poseEstimatorList;
    // PhotonCamera cameraList[] = VisionConstants.CameraReal.cameraList;

    private VisionData visionData;

    public Photonvision(VisionData visionData) {
        for (PhotonPoseEstimator poseEstimator : poseEstimatorList) {
            // Use MultiTag detection on the coprocessor, and fall back to the least
            // uncertain tag if that fails
            poseEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            // redundant, but why not (setting the correct apriltag size/model and correct
            // field layout)
            poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
            poseEstimator.setFieldTags(VisionConstants.aprilTagFieldLayout);
        }

        this.visionData = visionData;
    }

    public void cameraUpdatePose(int index) {
        PhotonCamera camera = cameraList[index];
        PhotonPoseEstimator poseEstimator = poseEstimatorList[index];

        List<PhotonPipelineResult> pipelineResults = camera.getAllUnreadResults();

        SmartDashboard.putBoolean(index + ": No targets", false);
        SmartDashboard.putBoolean(index + ": High Latency", false);
        SmartDashboard.putBoolean(index + ": single tag far", false);
        SmartDashboard.putBoolean(index + ": pose empty", false);

        // for each unused result in the pipeline
        for (PhotonPipelineResult pipelineResult : pipelineResults) {

            // skip if no tags found
            if (!pipelineResult.hasTargets()) {
                SmartDashboard.putBoolean(index + ": No targets", true);
                continue;
            }

            double latencyMillis = pipelineResult.metadata.getLatencyMillis();

            visionData.latencyMillis[index] = latencyMillis;
            visionData.targetsSeen[index] = pipelineResult.getTargets().size();

            // skip if latency is too high
            if (latencyMillis > VisionConstants.RejectionRequirements.maxLatencyMilliSec) {
                SmartDashboard.putBoolean(index + ": High Latency", true);

                continue;
            }

            if (pipelineResult.getTargets().size() == 1 &&
                    getHypotenuse(pipelineResult.getTargets().get(
                            0).bestCameraToTarget) > VisionConstants.RejectionRequirements.maxSingleTagDistanceMeters) {
                                
                SmartDashboard.putBoolean(index + ": single tag far", true);

                continue;
            }

            poseEstimator.setReferencePose(getReferencePose());
            var optional_robotPose = poseEstimator.update(pipelineResult);

            if (optional_robotPose.isEmpty()) {
                SmartDashboard.putBoolean(index + ": pose empty", true);

                continue;
            }

            Pose3d robotPose = optional_robotPose.get().estimatedPose;

            visionData.visionEstimatedPoses[index] = robotPose;

            updateStandardDeviations(pipelineResult);

            double timestamp = Timer.getFPGATimestamp() - latencyMillis / 1000.0;
            Robot.swerve.visionUpdateOdometry(robotPose.toPose2d(), timestamp);
        }
    }

    public void updatePose() {
        for (int i = 0; i < cameraList.length; i++) {
            cameraUpdatePose(i);
        }
    }

    public double getHypotenuse(Transform3d transform3d) {
        return Math.sqrt(
                Math.pow(transform3d.getX(), 2) + Math.pow(transform3d.getY(), 2) + Math.pow(transform3d.getZ(), 2));
    }

    public void updateStandardDeviations(PhotonPipelineResult result) {
        SwerveDrivePoseEstimator poseEstimator = Robot.swerve.getPoseEstimator();

        if (result.getTargets().size() == 0) {
            return;
        }
        if (result.getTargets().size() == 1) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(VisionConstants.StandardDeviations.OneTag.xy,
                    VisionConstants.StandardDeviations.OneTag.xy, VisionConstants.StandardDeviations.OneTag.thetaRads));
        }
        if (result.getTargets().size() == 2) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(VisionConstants.StandardDeviations.TwoTag.xy,
                    VisionConstants.StandardDeviations.TwoTag.xy,
                    VisionConstants.StandardDeviations.TwoTag.thetaRads));
        }
        if (result.getTargets().size() > 2) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(VisionConstants.StandardDeviations.ManyTag.xy,
                    VisionConstants.StandardDeviations.ManyTag.xy,
                    VisionConstants.StandardDeviations.ManyTag.thetaRads));
        }
    }
}
