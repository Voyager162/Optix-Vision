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

        public static enum TrigDirection 
        {
            LEFT,
            RIGHT,
            FORWARD
        }
        public static Pose2d buttonBoardSetpointMap(int index)
        {
            return ppSetpoints[index];
        }

        public static Pose2d reefTrig(Pose2d reefPose, TrigDirection direction) {
            double offsetMultiplier = 1;
            double forwardMagnitudeMultiplier = 1;
            double angleOffset = 90; //90 for perpindicular
            switch(direction)
            {
                case LEFT:
                offsetMultiplier = 1;
                break;

                case RIGHT:
                offsetMultiplier = -1;
                break;

                case FORWARD:
                    angleOffset = 0;
                    forwardMagnitudeMultiplier = 1.5;
                break;

                default:
                System.out.println("did not specify trig direction");
                break;
            }
    
            double xOffset = Math.cos(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))/(4*forwardMagnitudeMultiplier);
            double yOffset = Math.sin(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))/(4*forwardMagnitudeMultiplier);
            return new Pose2d(reefPose.getX()+xOffset,reefPose.getY()+yOffset,reefPose.getRotation()); 
            };
            public static final double ROBOT_LENGTH = 0.6; // Length of the robot in meters
            public static final double ROBOT_WIDTH = 0.6;  // Width of the robot in meters
            
            // Adjusted setpoints
            public static final Pose2d coralTop = adjustPose(0.851154, 7.39648, Math.toRadians(125));
            public static final Pose2d coralBottom = adjustPose(0.851154, 0.65532, Math.toRadians(-125));
            public static final Pose2d processor = adjustPose(5.987542, -0.00381, Math.toRadians(-90));
            
            public static final Pose2d reef18 = adjustPose(3.6576, 4.0259, Math.toRadians(0));
            public static final Pose2d reef17 = adjustPose(4.073906, 3.306318, Math.toRadians(60));
            public static final Pose2d reef22 = adjustPose(4.90474, 3.306318, Math.toRadians(120));
            public static final Pose2d reef21 = adjustPose(5.321046, 4.0259, Math.toRadians(180));
            public static final Pose2d reef20 = adjustPose(4.90474, 4.745482, Math.toRadians(-120));
            public static final Pose2d reef19 = adjustPose(4.073906, 4.745482, Math.toRadians(-60));
            
            // Helper method to adjust Pose2d
            public static Pose2d adjustPose(double x, double y, double heading) {
                // Calculate offsets based only on robot length
                double offsetX = (ROBOT_LENGTH / 2) * Math.cos(heading);
                double offsetY = (ROBOT_LENGTH / 2) * Math.sin(heading);
            
                // Adjust coordinates to align the robot’s front edge with the target
                return new Pose2d(x - offsetX, y - offsetY, new Rotation2d(heading));
            };

        public static final Pose2d reef17L = reefTrig(reef17, TrigDirection.LEFT);
        public static final Pose2d reef17R = reefTrig(reef17, TrigDirection.RIGHT);
        
        public static final Pose2d reef18L = reefTrig(reef18, TrigDirection.LEFT);
        public static final Pose2d reef18R = reefTrig(reef18, TrigDirection.RIGHT);

        public static final Pose2d reef19L = reefTrig(reef19, TrigDirection.LEFT);
        public static final Pose2d reef19R = reefTrig(reef19, TrigDirection.RIGHT);

        public static final Pose2d reef20L = reefTrig(reef20, TrigDirection.LEFT);
        public static final Pose2d reef20R = reefTrig(reef20, TrigDirection.RIGHT);

        public static final Pose2d reef21L = reefTrig(reef21, TrigDirection.LEFT);
        public static final Pose2d reef21R = reefTrig(reef21, TrigDirection.RIGHT);

        public static final Pose2d reef22L = reefTrig(reef22, TrigDirection.LEFT);
        public static final Pose2d reef22R = reefTrig(reef22, TrigDirection.RIGHT);

        // public static final Pose2d a = new Pose2d(3.76, 3.49, new Rotation2d(0));
        // public static final Pose2d b = new Pose2d(4.39, 4.93, new Rotation2d(0));
        // public static final Pose2d c = new Pose2d( 5.22,  4.57, new Rotation2d(0));
        // public static final Pose2d d = new Pose2d(3.76, 3.49, new Rotation2d(0));
        // public static final Pose2d e = new Pose2d(5.32, 3.67, new Rotation2d(0));
        // public static final Pose2d f = new Pose2d(4.59, 3.13, new Rotation2d(0));


        public static final Pose2d[] ppSetpoints = new Pose2d[] { 
            // ToPosConstants.Setpoints.a,
            // ToPosConstants.Setpoints.b,
            // ToPosConstants.Setpoints.c,
            // ToPosConstants.Setpoints.d,
            // ToPosConstants.Setpoints.e,
            // ToPosConstants.Setpoints.f,
            ToPosConstants.Setpoints.coralBottom,
            ToPosConstants.Setpoints.coralTop,
            ToPosConstants.Setpoints.processor, //index 2, and if you add or remove any setpoints, this will break on the fly,
            //read on the fly command for context
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