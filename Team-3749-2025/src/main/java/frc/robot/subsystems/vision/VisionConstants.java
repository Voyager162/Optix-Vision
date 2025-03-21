package frc.robot.subsystems.vision;

import java.util.function.Function;

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
                public static final double maxSingleTagDistanceMeters = Units.inchesToMeters(100);
        }

        // currently 3748's #'s'
        public class StandardDeviations {
                public class PreMatch {
                        public static final double xy = 0.001;
                        public static final double thetaRads = 0.0002;
                }

                public class OneTag {
                        public static final double slope =0.0021889 ;
                        public static final double thetaRads = Units.degreesToRadians(8);
                        public static final Function<Double, Double> regression = (distance) -> slope * distance;
                }

                public class TwoTag {
                        public static final double xy = Math.hypot(0.005, 0.008);
                        public static final double thetaRads = Units.degreesToRadians(3);
                }

                public class ManyTag {
                        public static final double xy = Math.hypot(0.002, 0.003);
                        public static final double thetaRads = Units.degreesToRadians(3);
                }
        }

        public class CameraPositions {
                public static Transform3d cam1 = new Transform3d(
                                Units.inchesToMeters(11.961),
                                Units.inchesToMeters(10.313),
                                Units.inchesToMeters(10.881),
                                new Rotation3d(0,
                                                Units.degreesToRadians(-15),
                                                Units.degreesToRadians(90)));

                public static Transform3d cam2 = new Transform3d(
                                Units.inchesToMeters(10.585),
                                Units.inchesToMeters(6.287),
                                Units.inchesToMeters(10.826),
                                new Rotation3d(Units.degreesToRadians(-1), //0
                                                Units.degreesToRadians(-10),
                                                Units.degreesToRadians(-30.1)));

                public static Transform3d cam3 = new Transform3d(
                                Units.inchesToMeters(9.504),

                                Units.inchesToMeters(-10.961),
                                Units.inchesToMeters(15.746),
                                new Rotation3d(0,
                                                Units.degreesToRadians(17),//20
                                                Units.degreesToRadians(0)));

                public static Transform3d cam4 = new Transform3d(
                                Units.inchesToMeters(-10.354),
                                Units.inchesToMeters(10.189),
                                Units.inchesToMeters(10.846),
                                new Rotation3d(0,
                                                Units.degreesToRadians(-15),
                                                Units.degreesToRadians(149.9)));

                public static Transform3d cam5 = new Transform3d(
                                Units.inchesToMeters(-12.244),
                                Units.inchesToMeters(6.74),
                                Units.inchesToMeters(10.814),
                                new Rotation3d(0,
                                                Units.degreesToRadians(-15),
                                                Units.degreesToRadians(45.1 + 180)));

                public static Transform3d cam6 = new Transform3d(
                                Units.inchesToMeters(-9.745),
                                Units.inchesToMeters(-11.177),
                                Units.inchesToMeters(20.27),
                                new Rotation3d(0,
                                                Units.degreesToRadians(-9.5),
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
                        .loadField(AprilTagFields.k2025ReefscapeWelded);
}
