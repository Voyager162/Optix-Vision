package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
public class ElevatorConstants {
    public static class ElevatorSpecs {
        public static final double gearing = 6.0;
        public static final double carriageMassKg = 2;
        public static final double drumRadiusMeters = 12;
        public static final double minHeightMeters = 0;
        public static final double maxHeightMeters = Units.feetToMeters(6);
        // public static final boolean simulateGravity = true;
        public static final double startingHeightMeters = 0;
    } 

    public static class ElevatorControl {
        public static final double kPSim = 0;
        public static final double kDSim = 0;
        public static final double kASim = 0;
        public static final double kGSim = 0;
        // public static final double kSSim = 0;
    }

    public enum ElevatorStates{
        STOP,
        GO,
        L1,
        L2,
        L3,
        L4,
        MAX
    }
}
