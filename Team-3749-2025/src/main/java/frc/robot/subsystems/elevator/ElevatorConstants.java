package frc.robot.subsystems.elevator;

import frc.robot.utils.LoggedTunableNumber;

/**
 * Elevator constants
 * 
 * @author Dhyan Soni
 * @author Andrew Ge
 */

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static class ElevatorSpecs {
        public static final double gearing = 16 * (24.0 / 22.0);
        public static final double carriageMassKg = Units.lbsToKilograms(54);
        // 22 TOOTH, 1/4 in pitch, divide by 2pi to go from circumfrence to radius
        public static final double drumRadiusMeters = (Units.inchesToMeters(22.0 / 4.0) / (2 * Math.PI));
        public static final double minHeightMeters = 0;
        public static final double maxHeightMeters = Units.feetToMeters(6); // remeasure maxV and A
        // public static final boolean simulateGravity = true;
        public static final double startingHeightMeters = 0;

        public static final double baseHeight = Units.feetToMeters(3.25);

        public static int[] motorIds = { 19, 20 };
        public static boolean[] motorInverted = { true, false };

        public static int zeroOffset = 0;
    }

    public static class ElevatorControl {
        public static LoggedTunableNumber kG = new LoggedTunableNumber("/subsystems/elevator/kG", 0.31);
        public static LoggedTunableNumber kP = new LoggedTunableNumber("/subsystems/elevator/kP", 0);
        public static LoggedTunableNumber kI = new LoggedTunableNumber("/subsystems/elevator/kI", 0);
        public static LoggedTunableNumber kD = new LoggedTunableNumber("/subsystems/elevator/kD", 0);
        public static LoggedTunableNumber kS = new LoggedTunableNumber("/subsystems/elevator/kS", 0.14);
        public static LoggedTunableNumber kV = new LoggedTunableNumber("/subsystems/elevator/kV", 7.821);
        public static LoggedTunableNumber kA = new LoggedTunableNumber("/subsystems/elevator/kA", 1.72);
        public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("/subsystems/elevator/max velocity",
                1.43);
        public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                "/subsystems/elevator/max acceleration",
                3.18);
    }

    public enum ElevatorStates {
        STOP,
        L1,
        L2,
        L3,
        L4,
        SOURCE,
        ALGAE_LOW,
        ALGAE_HIGH,
        MAX,
        STOW
    }

    public static class StateHeights {
        public static final double l1Height = Units.inchesToMeters(18);
        public static final double l2Height = Units.inchesToMeters(30);
        public static final double l3Height = Units.inchesToMeters(40);
        public static final double l4Height = Units.inchesToMeters(53.5);
        public static final double algaeLowHeight = Units.inchesToMeters(31.875);
        public static final double algaeHighHeight = Units.inchesToMeters(47.625);
        public static final double sourceHeight = Units.inchesToMeters(ElevatorSpecs.baseHeight);// check later
        public static final double stowHeight = Units.inchesToMeters(0.0); // placedholder
    }
}
