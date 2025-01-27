package frc.robot.subsystems.swerve;

import java.security.DrbgParameters.Capability;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ToPosConstants {

    // private final double hexOffset = SwerveConstants.DriveConstants.trackWidth /
    // 2.0;
    // private final double hexOffset = SwerveConstants.DriveConstants.trackWidth /
    // 2.0;

    public static final class PathPlannerConstants {
        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final double maxAngularVelocity = 0;
        public static final double maxAngularAcceleration = 0;
    }

    public static final class Setpoints {

        public static final double approachPointDistance = 1;

        public static enum TrigDirection {
            LEFT,
            RIGHT,
            FORWARD
        }

        // public static Pose2d buttonBoardSetpointMap(int index) {
        // return ppSetpoints[index];
        // }

        // public static Pose2d buttonBoardApproachSetpointMap(int index) {
        // return ppApproachSetpoints[index];
        // }

        public static Pose2d reefTrig(Pose2d reefPose, TrigDirection direction) {
            double offsetMultiplier = 1;
            double forwardMagnitudeMultiplier = 1;
            double angleOffset = 90; // 90 for perpindicular
            switch (direction) {
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

            double xOffset = Math
                    .cos(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))
                    / (4 * forwardMagnitudeMultiplier);
            double yOffset = Math
                    .sin(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))
                    / (4 * forwardMagnitudeMultiplier);
            return new Pose2d(reefPose.getX() + xOffset, reefPose.getY() + yOffset, reefPose.getRotation());
        };

        // Helper method to adjust Pose2d
        public static Pose2d adjustPose(double x, double y, double heading) {
            // Calculate offsets based only on robot length
            double offsetX = (ROBOT_LENGTH / 2) * Math.cos(heading);
            double offsetY = (ROBOT_LENGTH / 2) * Math.sin(heading);

            // Adjust coordinates to align the robot’s front edge with the target
            return new Pose2d(x - offsetX, y - offsetY, new Rotation2d(heading));
        };

        private static Pose2d createApproachPoint(Pose2d pose) {
            Translation2d position = pose.getTranslation();
            Rotation2d heading = pose.getRotation();
            Translation2d offset = new Translation2d(-approachPointDistance * Math.cos(heading.getRadians()),
                    -approachPointDistance * Math.sin(heading.getRadians()));
            return new Pose2d(position.plus(offset), heading);
        }

        public static final double ROBOT_LENGTH = .888; // Length of the robot in meters
        public static final double ROBOT_WIDTH = .888; // Width of the robot in meters

        // Adjusted setpoints
        public static final Pose2d coralLeft = adjustPose(0.851154+2*ROBOT_LENGTH, 7.39648+2*ROBOT_LENGTH, Math.toRadians(-55));
        public static final Pose2d coralRight = adjustPose(0.851154, 0.65532, Math.toRadians(55));
        public static final Pose2d processor = adjustPose(5.987542, -0.00381, Math.toRadians(-90));

        public static final Pose2d reefClose = adjustPose(3.65, 4, Math.toRadians(0));
        public static final Pose2d reefCloseRight = adjustPose(4.07, 3.25, Math.toRadians(60));
        public static final Pose2d reefFarRight = adjustPose(4.94, 3.25, Math.toRadians(120));
        public static final Pose2d reefFar = adjustPose(5.35, 4, Math.toRadians(180));
        public static final Pose2d reefFarLeft = adjustPose(4.94, 4.74, Math.toRadians(-120));
        public static final Pose2d reefCloseLeft = adjustPose(4.07, 4.74, Math.toRadians(-60));

        // Please refer to:
        // https://firstfrc.blob.core.windows.net/frc2025/Manual/Sections/2025GameManual-05ARENA.pdf
        // to see what each letter setpoint refers to

        public static final Pose2d aSetpoint = reefTrig(reefClose, TrigDirection.LEFT);
        public static final Pose2d bSetpoint = reefTrig(reefClose, TrigDirection.RIGHT);

        public static final Pose2d cSetpoint = reefTrig(reefCloseRight, TrigDirection.LEFT);
        public static final Pose2d dSetpoint = reefTrig(reefCloseRight, TrigDirection.RIGHT);

        public static final Pose2d eSetpoint = reefTrig(reefFarRight, TrigDirection.LEFT);
        public static final Pose2d fSetpoint = reefTrig(reefFarRight, TrigDirection.RIGHT);

        public static final Pose2d gSetpoint = reefTrig(reefFar, TrigDirection.LEFT);
        public static final Pose2d hSetpoint = reefTrig(reefFar, TrigDirection.RIGHT);

        public static final Pose2d iSetpoint = reefTrig(reefFarLeft, TrigDirection.LEFT);
        public static final Pose2d jSetpoint = reefTrig(reefFarLeft, TrigDirection.RIGHT);

        public static final Pose2d kSetpoint = reefTrig(reefCloseLeft, TrigDirection.LEFT);
        public static final Pose2d lSetpoint = reefTrig(reefCloseLeft, TrigDirection.RIGHT);

        public static final Pose2d aApproach = createApproachPoint(aSetpoint);
        public static final Pose2d bApproach = createApproachPoint(bSetpoint);
        public static final Pose2d cApproach = createApproachPoint(cSetpoint);
        public static final Pose2d dApproach = createApproachPoint(dSetpoint);
        public static final Pose2d eApproach = createApproachPoint(eSetpoint);
        public static final Pose2d fApproach = createApproachPoint(fSetpoint);
        public static final Pose2d gApproach = createApproachPoint(gSetpoint);
        public static final Pose2d hApproach = createApproachPoint(hSetpoint);
        public static final Pose2d iApproach = createApproachPoint(iSetpoint);
        public static final Pose2d jApproach = createApproachPoint(jSetpoint);
        public static final Pose2d kApproach = createApproachPoint(kSetpoint);
        public static final Pose2d lApproach = createApproachPoint(lSetpoint);

        public enum PPSetpoints {

            CORALLEFT(coralLeft, coralLeft),
            CORALRIGHT(coralRight, coralRight),
            A(aSetpoint, aApproach),
            B(bSetpoint, bApproach),
            C(cSetpoint, cApproach),
            D(dSetpoint, dApproach),
            E(eSetpoint, eApproach),
            F(fSetpoint, fApproach),
            G(gSetpoint, gApproach),
            H(hSetpoint, hApproach),
            I(iSetpoint, iApproach),
            J(jSetpoint, jApproach),
            K(kSetpoint, kApproach),
            L(lSetpoint, lApproach);

            public final Pose2d setpoint;
            public final Pose2d approachPoint;

            private PPSetpoints(Pose2d setpoint, Pose2d approachPoint) {
                this.setpoint = setpoint;
                this.approachPoint = approachPoint;
            }

        }

        // public static final Pose2d[] ppSetpoints = new Pose2d[] {
        // coralBottom,
        // coralTop,
        // processor, // index 2, and if you add or remove any setpoints, this will
        // break
        // // on the fly,
        // // read on the fly command for context
        // A,
        // B,
        // C,
        // D,
        // E,
        // F,
        // G,
        // H,
        // I,
        // J,
        // K,
        // L
        // };
        // public static final Pose2d[] ppApproachSetpoints = new Pose2d[] {
        // coralBottom,
        // coralTop,
        // processor, // index 2, and if you add or remove any setpoints, this will
        // break
        // // on the fly,
        // // read on the fly command for context
        // aApproach,
        // bApproach,
        // cApproach,
        // dApproach,
        // eApproach,
        // fApproach,
        // gApproach,
        // hApproach,
        // iApproach,
        // jApproach,
        // kApproach,
        // lApproach,
        // };

        // public static final Pose2d a = new Pose2d(3.76, 3.49, new Rotation2d(0));
        // public static final Pose2d b = new Pose2d(4.39, 4.93, new Rotation2d(0));
        // public static final Pose2d c = new Pose2d( 5.22, 4.57, new Rotation2d(0));
        // public static final Pose2d c = new Pose2d( 5.22, 4.57, new Rotation2d(0));
        // public static final Pose2d d = new Pose2d(3.76, 3.49, new Rotation2d(0));
        // public static final Pose2d e = new Pose2d(5.32, 3.67, new Rotation2d(0));
        // public static final Pose2d f = new Pose2d(4.59, 3.13, new Rotation2d(0));
        // these aren't reef letter id's just verticies of the hexagon

    }

    public static final class Obstacles {

    }

}