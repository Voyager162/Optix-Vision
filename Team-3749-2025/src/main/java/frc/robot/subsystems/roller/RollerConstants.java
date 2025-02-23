package frc.robot.subsystems.roller;

/**
 * Constants for roller subsystem
 * 
 * @author Lilian Wu
 */

import frc.robot.utils.LoggedTunableNumber;

public class RollerConstants {
        public static final class Algae {
                public static final int motorId = 22;
                public static final boolean inverted = false;

        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 9.0;
        public static final double measurementNoise = 0.0;

                public static LoggedTunableNumber kPVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/kPVelocity", 0.0);
                public static LoggedTunableNumber kIVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/kIVelocity", 0.0);
                public static LoggedTunableNumber kDVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/kDVelocity", 0.0);
                public static LoggedTunableNumber kSVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/kSVelocity", 0.35);
                public static LoggedTunableNumber kVVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/kVVelocity", 0.042);
                public static LoggedTunableNumber kAVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/kAVelocity", 0.0124);

                public static LoggedTunableNumber kPPosition = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/kPPosition", 0.0);
                public static LoggedTunableNumber kIPosition = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/kIPosition", 0.0);
                public static LoggedTunableNumber kDPosition = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/kDPosition", 0.0);

                public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/maxVelocity", 256);
                public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                                "/subsystems/rollers/algaeRoller/maxAcceleration", 0.0);

        }

        public static final class Coral {
                public static final int motorId = 15;
                public static final boolean inverted = true;

                public static final double momentOfInertia = 0.04;
                public static final double gearRatio = 3;
                public static final double measurementNoise = 0.0;

                public static LoggedTunableNumber kPVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/kPVelocity", 0.0);
                public static LoggedTunableNumber kIVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/kIVelocity", 0.0);
                public static LoggedTunableNumber kDVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/kDVelocity", 0.0);
                public static LoggedTunableNumber kSVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/kSVelocity", 0.205);
                public static LoggedTunableNumber kVVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/kVVelocity", 0.0594);
                public static LoggedTunableNumber kAVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/kAVelocity", 0.04);

                public static LoggedTunableNumber kPPosition = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/kPPosition", 0.5);
                public static LoggedTunableNumber kIPosition = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/kIPosition", 0.0);
                public static LoggedTunableNumber kDPosition = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/kDPosition", 0.0);

                public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/maxVelocity", 194.0);
                public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/maxAcceleration", 415.0);

                public static LoggedTunableNumber intakeVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/intakeVelocitySetpoint", 150.0);
                public static LoggedTunableNumber outtakeVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/coralRoller/outtakeVelocitySetpoint", -40.0);

        }

        public static final class Scoring {
                public static final int motorId = 21;
                public static final boolean inverted = false;

                public static final double momentOfInertia = 0.04;
                public static final double gearRatio = 4;
                public static final double measurementNoise = 0.0;

                public static LoggedTunableNumber kPVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/kPVelocity", 0.0);
                public static LoggedTunableNumber kIVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/kIVelocity", 0.0);
                public static LoggedTunableNumber kDVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/kDVelocity", 0.0);
                public static LoggedTunableNumber kSVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/kSVelocity", 0.14);
                public static LoggedTunableNumber kVVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/kVVelocity", 0.0619);
                public static LoggedTunableNumber kAVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/kAVelocity", 0.0142);

                public static LoggedTunableNumber kPPosition = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/kPPosition", .25);
                public static LoggedTunableNumber kIPosition = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/kIPosition", 0.0);
                public static LoggedTunableNumber kDPosition = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/kDPosition", 0.0);

                public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/maxVelocity", 194);
                public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                                "/subsystems/rollers/scoringRoller/maxAcceleration", 412);

                public static final int sensorPort = 7;

                public static final double reverseDistance = 7;
        }

        public enum RollerStates {
                INTAKE(50, 80, 200),
                MAINTAIN(0, 0, 0),
                OUTTAKE(-50, -30, 200),
                STOP(0, 0, 0);

                public final double algaeVelocity;
                public final double coralVelocity;
                public final double scoringVelocity;

                private RollerStates(double algae, double coral, double scoring) {
                        algaeVelocity = algae;
                        coralVelocity = coral;
                        scoringVelocity = scoring;
                }

        }

        public enum Implementations {
                ALGAE,
                CORAL,
                SCORING
        }
}
