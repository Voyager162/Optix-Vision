package frc.robot.subsystems.vision;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public class CameraPositions {
        public static Transform3d cam1 = new Transform3d(0, 0, 0, null);
        public static Transform3d cam2 = new Transform3d(0, 0, 0, null);
        public static Transform3d cam3 = new Transform3d(0, 0, 0, null);
        public static Transform3d cam4 = new Transform3d(0, 0, 0, null);
        public static Transform3d cam5 = new Transform3d(0, 0, 0, null);
        public static Transform3d cam6 = new Transform3d(0, 0, 0, null);

        public static Transform3d[] cameraList = { cam1, cam2, cam3, cam4, cam5, cam6 };
    }

    public class CameraReal {
        public static final PhotonCamera cam1 = new PhotonCamera("cam1");
        public static final PhotonCamera cam2 = new PhotonCamera("cam2");
        public static final PhotonCamera cam3 = new PhotonCamera("cam3");
        public static final PhotonCamera cam4 = new PhotonCamera("cam4");
        public static final PhotonCamera cam5 = new PhotonCamera("cam5");
        public static final PhotonCamera cam6 = new PhotonCamera("cam6");

        public static final PhotonCamera[] cameraList = { cam1, cam2, cam3, cam4, cam5, cam6 };

        public static PhotonPoseEstimator poseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam1);
        public static PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam2);
        public static PhotonPoseEstimator poseEstimator3 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam3);
        public static PhotonPoseEstimator poseEstimator4 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam4);
        public static PhotonPoseEstimator poseEstimator5 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam5);
        public static PhotonPoseEstimator poseEstimator6 = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CameraPositions.cam6);

        public static PhotonPoseEstimator[] poseEstimatorList = { poseEstimator1, poseEstimator2, poseEstimator3,
                poseEstimator4, poseEstimator5, poseEstimator6 };
    }

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025Reefscape);
}
