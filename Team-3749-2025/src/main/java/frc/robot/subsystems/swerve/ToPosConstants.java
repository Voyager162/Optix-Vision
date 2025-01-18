package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ToPosConstants {

    //private final double hexOffset = SwerveConstants.DriveConstants.trackWidth / 2.0;

    public static final class PathPlannerConstants {

        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final double maxAngularVelocity = 0;
        public static final double maxAngularAcceleration = 0;
    }

    public static final class Setpoints {

        public static Pose2d buttonBoardSetpointMap(int index)
        {
            return ppSetpoints[index];
        }

        static Pose2d reefTrig(Pose2d reefPose, boolean isRight) {
            double offsetMultiplier = 1;
            if (isRight) {
                offsetMultiplier = -1;
            }
    
            double xOffset = Math.cos(Math.toRadians(reefPose.getRotation().getDegrees() + (90 * offsetMultiplier)))/4;
            double yOffset = Math.sin(Math.toRadians(reefPose.getRotation().getDegrees() + (90 * offsetMultiplier)))/4;
            return new Pose2d(reefPose.getX()+xOffset,reefPose.getY()+yOffset,reefPose.getRotation());
            };

        public static final Pose2d coralTop = new Pose2d(1.21, 7.0, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d coralBottom = new Pose2d(1.21, 1.15, new Rotation2d(Math.toRadians(231)));
        public static final Pose2d processor = new Pose2d(6.3, 0.66, new Rotation2d(Math.toRadians(-90)));

        public static final Pose2d reef18 = new Pose2d(3.09, 4.03, new Rotation2d(Math.toRadians(0)));
        public static final Pose2d reef17 = new Pose2d(3.7, 2.85, new Rotation2d(Math.toRadians(60)));

        public static final Pose2d reef22 = new Pose2d(5.24, 2.75, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d reef21 = new Pose2d(6, 4, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d reef20 = new Pose2d(5.22, 5.37, new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d reef19 = new Pose2d(3.73, 5.24, new Rotation2d(Math.toRadians(-60)));

        public static final Pose2d reef17L = reefTrig(reef17, false);
        public static final Pose2d reef17R = reefTrig(reef17, true);
        
        public static final Pose2d reef18L = reefTrig(reef18, false);
        public static final Pose2d reef18R = reefTrig(reef18, true);

        public static final Pose2d reef19L = reefTrig(reef19, false);
        public static final Pose2d reef19R = reefTrig(reef19, true);

        public static final Pose2d reef20L = reefTrig(reef20, false);
        public static final Pose2d reef20R = reefTrig(reef20, true);

        public static final Pose2d reef21L = reefTrig(reef21, false);
        public static final Pose2d reef21R = reefTrig(reef21, true);

        public static final Pose2d reef22L = reefTrig(reef22, false);
        public static final Pose2d reef22R = reefTrig(reef22, true);

        public static final Pose2d[] ppSetpoints = new Pose2d[] { 
            ToPosConstants.Setpoints.coralBottom,
            ToPosConstants.Setpoints.coralTop,
            ToPosConstants.Setpoints.processor,
            ToPosConstants.Setpoints.reef17L,
            ToPosConstants.Setpoints.reef17R,
            ToPosConstants.Setpoints.reef18L,
            ToPosConstants.Setpoints.reef18R,
            ToPosConstants.Setpoints.reef19L,
            ToPosConstants.Setpoints.reef19R,
            ToPosConstants.Setpoints.reef20L,
            ToPosConstants.Setpoints.reef20R,
            ToPosConstants.Setpoints.reef21L,
            ToPosConstants.Setpoints.reef21R,
            ToPosConstants.Setpoints.reef22L,
            ToPosConstants.Setpoints.reef22R,
        };
    }

    public static final class Obstacles {

    }

}