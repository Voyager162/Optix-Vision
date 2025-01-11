package frc.robot.subsystems.arm;

/**
 * Constants file for the arm subsystem
 * 
 * @author Weston Gardner
 * 
 */

public class ArmConstants {
    public static final int motorId = 0;

    public static final int armLength_inches = 8;
    public static final double armLength_meters = 0.2032;

    public static final int armMinAngle_degrees = 90;
    public static final int armMaxAngle_degrees = 180;
    public static final int armStartingAngle_degrees = 90;

    public static final double armMass_kg = 5.0;
    public static final double armGearing = 255;

    public static double kPSim = 50; // extremely high but works in simulation
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;

    public static double kGSim = 0.494286; // a little specific but hey, why not
    public static final double kSSim = 0.0;
    public static final double kASim = 0.0;
    public static final double kVSim = 0.0;

    public static final double stowSetPoint = Math.PI / 2;
    public static final double halfwayExtendedSetPoint = Math.PI * 3 / 4;
    public static final double fullyExtendedSetPoint = Math.PI;

    public enum ArmStates {
        MOVING_DOWN,
        MOVING_UP,
        HALFWAY_EXTENDED,
        FULLY_EXTENDED,
        STOWED,
        STOPPED
    }
}
