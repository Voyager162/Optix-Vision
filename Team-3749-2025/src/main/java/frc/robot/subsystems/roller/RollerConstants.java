package frc.robot.subsystems.roller;

public class RollerConstants {
    public static final class Algae {
        public static final int motorId = 1;
        public static final boolean inverted = false;

        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 1.0;
        public static final double measurementNoise = 0.0;

        public static double kPVelocity = 10.0;
        public static double kIVelocity = 0.0;
        public static double kDVelocity = 0.0;
        public static double kSVelocity = 0.0;
        public static double kVVelocity = 0.02;
        public static double kAVelocity = 0.0;

        public static double kPPosition = 15.0;
        public static double kIPosition = 0.0;
        public static double kDPosition = 10.0;

        public static final double velocity = 5.0;

        public static double maxVelocity = 0.0;
        public static double maxAcceleration = 0.0;
    }

    public static final class Coral {
        public static final int motorId = 18;
        public static final boolean inverted = false;

        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 1.0;
        public static final double measurementNoise = 0.0;

        public static double kPVelocity = 10.0;
        public static double kIVelocity = 0.0;
        public static double kDVelocity = 0.0;
        public static double kSVelocity = 0.0;
        public static double kVVelocity = 0.02;
        public static double kAVelocity = 0.0;

        public static double kPPosition = 15.0;
        public static double kIPosition = 0.0;
        public static double kDPosition = 10.0;

        public static final double velocity = 5.0;

        public static double maxVelocity = 0.0;
        public static double maxAcceleration = 0.0;
    }

    public static final class Scoring {
        public static final int motorId = 21;
        public static final boolean inverted = false;

        public static final double momentOfInertia = 0.04;
        public static final double gearRatio = 1.0;
        public static final double measurementNoise = 0.0;

        public static double kPVelocity = 10.0;
        public static double kIVelocity = 0.0;
        public static double kDVelocity = 0.0;
        public static double kSVelocity = 0.0;
        public static double kVVelocity = 0.02;
        public static double kAVelocity = 0.0;

        public static double kPPosition = 15.0;
        public static double kIPosition = 0.0;
        public static double kDPosition = 10.0;

        public static final double velocity = 5.0;

        public static double maxVelocity = 0.0;
        public static double maxAcceleration = 0.0;
    }

    public enum RollerStates {
        RUN,
        MAINTAIN,
        STOP
    }

    public enum Implementations {
        ALGAE,
        CORAL,
        SCORING
    }
}
