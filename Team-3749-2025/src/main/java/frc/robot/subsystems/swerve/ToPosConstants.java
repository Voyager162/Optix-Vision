package frc.robot.subsystems.swerve;

import java.util.HashMap;
import java.util.List;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;

public class ToPosConstants {

    // private final double hexOffset = SwerveConstants.DriveConstants.trackWidth /
    // 2.0;
    // private final double hexOffset = SwerveConstants.DriveConstants.trackWidth /
    // 2.0;

    public static final ChoreoAllianceFlipUtil.Flipper flipper = ChoreoAllianceFlipUtil.getFlipper();

    public static Pose2d flipPose(Pose2d pose) {
        Translation2d translation = new Translation2d(flipper.flipX(pose.getX()), flipper.flipY(pose.getY()));
        Rotation2d rotation = new Rotation2d(flipper.flipHeading(pose.getRotation().getRadians()));
        return new Pose2d(translation, rotation);
    }

    public static final class ReefVerticies {

        public static final double positionTolerance = .5; // meters
        public static final double rotationTolerance = 10; // degrees

        // MAKE CONSTANTS
        private static final double SAFE_MARGIN = .95; // Safety margin around the robot.
        private static final double xComponenet = Math.cos(Math.toRadians(30));
        private static final double yComponenet = Math.sin(Math.toRadians(30));

        private static Translation2d flipIfRed(Translation2d translation) {
            if (DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red1)) {
                return new Translation2d(flipper.flipX(translation.getX()), flipper.flipY(translation.getY()));
            }
            return translation;
        }
        // Vertices of the hexagon, adjusted for safety margins.

        public static List<Translation2d> HEXAGON_VERTICES = List.of(
                flipIfRed(new Translation2d(3.668 - xComponenet * SAFE_MARGIN, 3.520 - yComponenet * SAFE_MARGIN)), // close
                                                                                                                    // right
                flipIfRed(new Translation2d(4.5, 3.039 - SAFE_MARGIN)), // middle right
                flipIfRed(new Translation2d(5.332 + xComponenet * SAFE_MARGIN, 3.520 - yComponenet * SAFE_MARGIN)), // far
                                                                                                                    // right
                flipIfRed(new Translation2d(5.332 + xComponenet * SAFE_MARGIN, 4.480 + yComponenet * SAFE_MARGIN)), // far
                                                                                                                    // left
                flipIfRed(new Translation2d(4.5, 4.961 + SAFE_MARGIN)), // middle left
                flipIfRed(new Translation2d(3.668 - xComponenet * SAFE_MARGIN, 4.480 + yComponenet * SAFE_MARGIN)), // close
                                                                                                                    // left
                flipIfRed(new Translation2d(3.668 - xComponenet * SAFE_MARGIN, 3.520 - yComponenet * SAFE_MARGIN))); // close
                                                                                                                     // right
    }

    public static final class PathPlannerConstants {
        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;
        public static final double maxAngularVelocity = 0;
        public static final double maxAngularAcceleration = 0;
    }

    public static final class Setpoints {

        public static final double approachPointDistance = 0.6;

        public static enum TrigDirection {
            LEFT,
            RIGHT,
            BACKWARD
        }

        public static Pose2d reefTrig(Pose2d reefPose, TrigDirection direction) {
            double offsetMultiplier = 1;
            double isBackwardOffset = 1;
            double angleOffset = 90; // 90 for perpindicular
            double distance = 6.5;
            switch (direction) {
                case LEFT:
                    offsetMultiplier = 1;
                    break;

                case RIGHT:
                    offsetMultiplier = -1;
                    break;
                case BACKWARD:
                    angleOffset = 0;
                    distance=3;
                    isBackwardOffset=-1;
                break;
            }

            double xSetup = reefPose.getX() + Math
                    .cos(Math.toRadians(reefPose.getRotation().getDegrees() + (90))) * Units.inchesToMeters(6.25);
            double ySetup = reefPose.getY() + Math
                    .sin(Math.toRadians(reefPose.getRotation().getDegrees() + (90))) * Units.inchesToMeters(6.25);

            double newX = xSetup + Math
                    .cos(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))
                    * Units.inchesToMeters(distance) * isBackwardOffset;
            double newY = ySetup + Math
                    .sin(Math.toRadians(reefPose.getRotation().getDegrees() + (angleOffset * offsetMultiplier)))
                    * Units.inchesToMeters(distance) * isBackwardOffset;

            return new Pose2d(newX, newY, reefPose.getRotation());
        };

        // Helper method to adjust Pose2d
        public static Pose2d adjustPose(double x, double y, double heading, boolean isCoralStation) {
            // Calculate offsets based only on robot length
            double offsetX = (ROBOT_LENGTH / 2) * Math.cos(heading);
            double offsetY = (ROBOT_WIDTH / 2) * Math.sin(heading);

            // Adjust coordinates to align the robot’s front edge with the target
            if (isCoralStation) {
                return new Pose2d(x + offsetX, y + offsetY, new Rotation2d(heading));

            }

            return new Pose2d(x - offsetX, y - offsetY, new Rotation2d(heading));
        };

        private static Pose2d createApproachPoint(Pose2d pose) {
            Translation2d position = pose.getTranslation();
            Rotation2d heading = pose.getRotation();
            Translation2d offset = new Translation2d(-approachPointDistance * Math.cos(heading.getRadians()),
                    -approachPointDistance * Math.sin(heading.getRadians()));
            return new Pose2d(position.plus(offset), heading);
        }

        private static Pose2d rotatePose(Pose2d pose, double degrees) {
            return new Pose2d(pose.getX(), pose.getY(),
                    new Rotation2d(Math.toRadians(pose.getRotation().getDegrees() + degrees)));
        }

        public static final double ROBOT_LENGTH = Units.inchesToMeters(37); // Length of the robot in meters
        public static final double ROBOT_WIDTH = Units.inchesToMeters(30);; // Width of the robot in meters

        // Adjusted setpoints
        public static Pose2d coralLeft = adjustPose(0.851154, 7.39648, Math.toRadians(-55), true);
        public static Pose2d coralRight = adjustPose(0.851154, 0.65532, Math.toRadians(55), true);
        public static Pose2d processor = adjustPose(5.987542, -0.00381, Math.toRadians(-90), false);

        public static Pose2d reefClose = adjustPose(3.65, 4, Math.toRadians(0), false);
        public static Pose2d reefCloseRight = adjustPose(4.07, 3.25, Math.toRadians(60), false);
        public static Pose2d reefFarRight = adjustPose(4.94, 3.25, Math.toRadians(120), false);
        public static Pose2d reefFar = adjustPose(5.35, 4, Math.toRadians(180), false);
        public static Pose2d reefFarLeft = adjustPose(4.94, 4.74, Math.toRadians(-120), false);
        public static Pose2d reefCloseLeft = adjustPose(4.07, 4.74, Math.toRadians(-60), false);

        // Please refer to:
        // https://firstfrc.blob.core.windows.net/frc2025/Manual/Sections/2025GameManual-05ARENA.pdf
        // to see what each letter setpoint refers to

        public static Pose2d aSetpoint = reefTrig(reefClose, TrigDirection.LEFT);
        public static Pose2d aL1 = reefTrig(rotatePose(aSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d bSetpoint = reefTrig(reefClose, TrigDirection.RIGHT);
        public static Pose2d bL1 = reefTrig(rotatePose(bSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d cSetpoint = reefTrig(reefCloseRight, TrigDirection.LEFT);
        public static Pose2d cL1 = reefTrig(rotatePose(cSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d dSetpoint = reefTrig(reefCloseRight, TrigDirection.RIGHT);
        public static Pose2d dL1 = reefTrig(rotatePose(dSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d eSetpoint = reefTrig(reefFarRight, TrigDirection.LEFT);
        public static Pose2d eL1 = reefTrig(rotatePose(eSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d fSetpoint = reefTrig(reefFarRight, TrigDirection.RIGHT);
        public static Pose2d fL1 = reefTrig(rotatePose(fSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d gSetpoint = reefTrig(reefFar, TrigDirection.LEFT);
        public static Pose2d gL1 = reefTrig(rotatePose(gSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d hSetpoint = reefTrig(reefFar, TrigDirection.RIGHT);
        public static Pose2d hL1 = reefTrig(rotatePose(hSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d iSetpoint = reefTrig(reefFarLeft, TrigDirection.LEFT);
        public static Pose2d iL1 = reefTrig(rotatePose(iSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d jSetpoint = reefTrig(reefFarLeft, TrigDirection.RIGHT);
        public static Pose2d jL1 = reefTrig(rotatePose(jSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d kSetpoint = reefTrig(reefCloseLeft, TrigDirection.LEFT);
        public static Pose2d kL1 = reefTrig(rotatePose(kSetpoint, 90),TrigDirection.BACKWARD);

        public static Pose2d lSetpoint = reefTrig(reefCloseLeft, TrigDirection.RIGHT);
        public static Pose2d lL1 = reefTrig(rotatePose(lSetpoint, 90),TrigDirection.BACKWARD);

        public static List<Pose2d> reefSides = List.of(
                reefClose,
                reefCloseRight,
                reefCloseLeft,
                reefFar,
                reefFarLeft,
                reefFarRight);

        public static HashMap<Pose2d, int[]> driveRelativeBranches = new HashMap<Pose2d, int[]>() {
            {
                put(reefClose, new int[] { 2, 4 }); // L R
                put(reefCloseLeft, new int[] { 22, 24 });
                put(reefCloseRight, new int[] { 7, 8 });
                put(reefFarRight, new int[] { 12, 10 });
                put(reefFar, new int[] { 16, 14 });
                put(reefFarLeft, new int[] { 20, 18 });
            }
        };

        public static Pose2d aApproach = createApproachPoint(aSetpoint);
        public static Pose2d bApproach = createApproachPoint(bSetpoint);
        public static Pose2d cApproach = createApproachPoint(cSetpoint);
        public static Pose2d dApproach = createApproachPoint(dSetpoint);
        public static Pose2d eApproach = createApproachPoint(eSetpoint);
        public static Pose2d fApproach = createApproachPoint(fSetpoint);
        public static Pose2d gApproach = createApproachPoint(gSetpoint);
        public static Pose2d hApproach = createApproachPoint(hSetpoint);
        public static Pose2d iApproach = createApproachPoint(iSetpoint);
        public static Pose2d jApproach = createApproachPoint(jSetpoint);
        public static Pose2d kApproach = createApproachPoint(kSetpoint);
        public static Pose2d lApproach = createApproachPoint(lSetpoint);

        public enum PPSetpoints {
            CORALLEFT(coralLeft, coralLeft),
            CORALRIGHT(coralRight, coralRight),

            A(aSetpoint, aApproach),
            AL1(aL1, aApproach),

            B(bSetpoint, bApproach),
            BL1(bL1, bApproach),

            C(cSetpoint, cApproach),
            CL1(cL1, cApproach),

            D(dSetpoint, dApproach),
            DL1(dL1, dApproach),

            E(eSetpoint, eApproach),
            EL1(eL1, eApproach),

            F(fSetpoint, fApproach),
            FL1(fL1, fApproach),

            G(gSetpoint, gApproach),
            GL1(gL1, gApproach),

            H(hSetpoint, hApproach),
            HL1(hL1, hApproach),

            I(iSetpoint, iApproach),
            IL1(iL1, iApproach),

            J(jSetpoint, jApproach),
            JL1(jL1, jApproach),

            K(kSetpoint, kApproach),
            KL1(kL1, kApproach),

            L(lSetpoint, lApproach),
            LL1(lL1, lApproach),

            REEFCLOSE(rotatePose(reefClose,90), createApproachPoint(reefClose)),
            REEFCLOSELEFT(rotatePose(reefCloseLeft,90), createApproachPoint(reefCloseLeft)),
            REEFCLOSERIGHT(rotatePose(reefCloseRight,90), createApproachPoint(reefCloseRight)),

            REEFFAR(rotatePose(reefFar,90), createApproachPoint(reefFar)),
            REEFFARLEFT(rotatePose(reefFarLeft,90), createApproachPoint(reefFarLeft)),
            REEFFARRIGHT(rotatePose(reefFarRight,90), createApproachPoint(reefFarRight));

            public Pose2d setpoint;
            public Pose2d approachPoint;
            public Command onReachCommand;

            private PPSetpoints(Pose2d setpoint, Pose2d approachPoint) {
                this.setpoint = setpoint;
                this.approachPoint = approachPoint;
                if (DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red1)) {
                    this.setpoint = flipPose(setpoint);
                    this.approachPoint = flipPose(approachPoint);
                }
            }

        }

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