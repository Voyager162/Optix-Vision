package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ToPosConstants {
    public static final class PathPlannerConstants {
        
        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final double maxAngularVelocity = 0;
        public static final double maxAngularAcceleration = 0;
    }

    public static final class Setpoints
    {
        public static final Pose2d coralTop = new Pose2d(1.21, 7.0, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d coralBottom = new Pose2d(1.21,1.15,new Rotation2d(Math.toRadians(231)));
        public static final Pose2d processor = new Pose2d(11.3,7.3,new Rotation2d(Math.toRadians(93.75)));
        public static final Pose2d reef18 = new Pose2d(3.09,4.03,new Rotation2d(Math.toRadians(0)));
        public static final Pose2d reef17 = new Pose2d(3.7,2.85,new Rotation2d(Math.toRadians(60)));
        public static final Pose2d reef22 = new Pose2d(5.24,2.75,new Rotation2d(Math.toRadians(120)));
        public static final Pose2d reef21 = new Pose2d(6,4,new Rotation2d(Math.toRadians(180)));
        public static final Pose2d reef20 = new Pose2d(5.22,5.37,new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d reef19 = new Pose2d(3.73,5.24,new Rotation2d(Math.toRadians(-60)));
    }

    public static final class Obstacles
    {
        
    }
    
}