//! maybe just remove this? whats the point in simming cameras

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Robot;

public interface VisionIO {
    public PhotonPoseEstimator poseEstimatorList[] = VisionConstants.CameraReal.poseEstimatorList;
    public PhotonCamera cameraList[] = VisionConstants.CameraReal.cameraList;

    public static class VisionData {
        public Pose2d[] visionEstimatedPoses;
        public double[] latencyMillis = new double[cameraList.length];
        public double[] targetsSeen = new double[cameraList.length];
    }

    // public void updateData(VisionData data);

    public void updatePose();

    public void cameraUpdatePose(int index);

    public default Pose2d getReferencePose() {
        return Robot.swerve.getPose();
    }

    public default PhotonCamera getCamera(int index) {
        return cameraList[index];
    }
}
