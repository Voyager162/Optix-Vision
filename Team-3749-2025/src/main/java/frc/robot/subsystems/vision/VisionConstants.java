package frc.robot.subsystems.vision;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

        public class RejectionRequirements {
                public static final double maxLatencyMilliSec = 150;
                public static final double maxSingleTagDistanceMeters = 20;
        }

        // currently 3748's #'s'
        public class StandardDeviations {
                public class PreMatch {
                        public static final double xy = 0.001;
                        public static final double thetaRads = 0.0002;
                }

                public class OneTag {
                        public static final double xy = Math.hypot(0.15, 0.33);
                        public static final double thetaRads = Units.degreesToRadians(7);
                }

                public class TwoTag {
                        public static final double xy = Math.hypot(0.005, 0.008);
                        public static final double thetaRads = Units.degreesToRadians(2);
                }

                public class ManyTag {
                        public static final double xy = Math.hypot(0.002, 0.003);
                        public static final double thetaRads = Units.degreesToRadians(2);
                }
        }

        public class CameraPositions {
                public static Transform3d cam1 = new Transform3d(
                                Units.inchesToMeters(10.498),
                                Units.inchesToMeters(15.274),
                                Units.inchesToMeters(10.354),
                                new Rotation3d(0,
                                                Units.degreesToRadians(15),
                                                Units.degreesToRadians(90-22)));

                public static Transform3d cam2 = new Transform3d(
                                Units.inchesToMeters(11.339),
                                Units.inchesToMeters(11.366),
                                Units.inchesToMeters(10.357),
                                new Rotation3d(0,
                                                Units.degreesToRadians(15),
                                                Units.degreesToRadians(40 - 90)));

                public static Transform3d cam3 = new Transform3d(
                                Units.inchesToMeters(9.504),
                                Units.inchesToMeters(-10.961),
                                Units.inchesToMeters(15.746),
                                new Rotation3d(0,
                                                Units.degreesToRadians(-20),
                                                Units.degreesToRadians(0)));

                public static Transform3d cam4 = new Transform3d(
                                Units.inchesToMeters(-11.886),
                                Units.inchesToMeters(14.995),
                                Units.inchesToMeters(10.461),
                                new Rotation3d(0,
                                                Units.degreesToRadians(4.9),
                                                Units.degreesToRadians(30+90)));

                public static Transform3d cam5 = new Transform3d(
                                Units.inchesToMeters(-12.839),
                                Units.inchesToMeters(10.925),
                                Units.inchesToMeters(10.447),
                                new Rotation3d(0,
                                                Units.degreesToRadians(15),
                                                Units.degreesToRadians(35+180)));

                public static Transform3d cam6 = new Transform3d(
                                Units.inchesToMeters(-9.745),
                                Units.inchesToMeters(-11.177),
                                Units.inchesToMeters(20.27),
                                new Rotation3d(0,
                                                Units.degreesToRadians(10),
                                                Units.degreesToRadians(-260 + 90)));

                public static Transform3d[] cameraList = { cam1, cam2, cam3, cam4, cam5, cam6 };
        }

        public class CameraReal {
                public static final int numCameras = 6;
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

                public static PhotonPoseEstimator[] poseEstimatorList = { poseEstimator1, poseEstimator2,
                                poseEstimator3,
                                poseEstimator4, poseEstimator5, poseEstimator6 };
        }

        public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2025Reefscape);
}
