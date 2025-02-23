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
                        public static final double xy = Math.hypot(0.015, 0.033);
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
                public static Transform3d cam1 = new Transform3d(Units.inchesToMeters(11.612),
                                Units.inchesToMeters(14.891), Units.inchesToMeters(10.791),
                                new Rotation3d(0, Units.degreesToRadians(
                                                -15), Units.degreesToRadians(60)));

                public static Transform3d cam2 = new Transform3d(Units.inchesToMeters(10.403),
                                Units.inchesToMeters(11.783), Units.inchesToMeters(10.501),
                                new Rotation3d(0, 0, Units.degreesToRadians(-40)));

                public static Transform3d cam3 = new Transform3d(-9.187, -4.529, 4.614, new Rotation3d(0, 4.9, 10));

                public static Transform3d cam4 = new Transform3d(Units.inchesToMeters(-11.586),
                                Units.inchesToMeters(14.161), Units.inchesToMeters(10.711),
                                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(200)));

                public static Transform3d cam5 = new Transform3d(Units.inchesToMeters(-9.745),
                                Units.inchesToMeters(-11.177), Units.inchesToMeters(20.27),
                                new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-260)));

                public static Transform3d cam6 = new Transform3d(0, 0, 0, null);

                public static Transform3d[] cameraList = { cam1, cam2, cam3, cam4, cam5, cam6 };
        }

        public class CameraReal {
                public static final PhotonCamera cam1 = new PhotonCamera("1");
                public static final PhotonCamera cam2 = new PhotonCamera("2");
                public static final PhotonCamera cam3 = new PhotonCamera("3");
                public static final PhotonCamera cam4 = new PhotonCamera("4");
                public static final PhotonCamera cam5 = new PhotonCamera("5");
                public static final PhotonCamera cam6 = new PhotonCamera("6");

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

                public static PhotonPoseEstimator[] poseEstimatorList = { poseEstimator1, poseEstimator2,
                                poseEstimator3,
                                poseEstimator4, poseEstimator5, poseEstimator6 };
        }

        public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2025Reefscape);
}
