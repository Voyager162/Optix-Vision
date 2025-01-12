package frc.robot.subsystems.arm;

/**
 * Constants file for the arm subsystem
 * 
 * @author Weston Gardner
 * 
 */

public class ArmConstants {

    public class algeaArmConstants {
        public static final int motorId = 0;
        public static final int numMotors = 1;

        public static final int armLength_inches = 17;
        public static final double armLength_meters = armLength_inches / 39.3701; // 0.4318

        public static final int armMinAngle_degrees = 80;
        public static final int armMaxAngle_degrees = 210;
        public static final int armStartingAngle_degrees = 90;

        public static final double armMass_kg = 50.92; // rough estimate for a volume of 395.071 in^3, and the material being polycarbonate steel, density = 7.86 g/cm^3
        public static final double armGearing = 49; // rough estimate

        public static double kP = 4;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static double kG = 2.48513;
        public static final double kS = 0.0;
        public static final double kA = 0.0;
        public static final double kV = 0.0;

        public static final double stowSetPoint_rad = 80 * Math.PI / 180; // 80 degrees
        public static final double processorSetPoint_rad = Math.PI / 2; // 90 degrees
        public static final double algeaPickUpSetPoint_rad = 160 * Math.PI / 180; // 160 degrees

        public static final double maxVelocity = 2.4223698370699234;
        public static final double maxAcceleration = 0.0;

        public static final double momentOfInertia = 0.775; // from last years MOI
        public static final boolean simulateGravity = true;

        public enum ArmStates {
            MOVING_DOWN,
            MOVING_UP,
            STOWED,
            PROCESSOR,
            ALGEA_PICKUP,
            STOPPED
        }
    }

    public class coralArmConstants {
        public static final int motorId = 0;
        public static final int numMotors = 1;

        public static final int armLength_inches = 10; // rough estimate
        public static final double armLength_meters = armLength_inches / 39.3701;

        public static final int armMinAngle_degrees = 10;
        public static final int armMaxAngle_degrees = 180;
        public static final int armStartingAngle_degrees = 90;

        public static final double armMass_kg = 15.32;
        public static final double armGearing = 40;

        public static double kP = 2;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static double kG = 5.1752;
        public static final double kS = 0.0;
        public static final double kA = 0.0;
        public static final double kV = 0.0;

        public static final double stowSetPoint_rad = 70 * Math.PI / 180; // 70
        public static final double handOffSetPoint_rad = 40 * Math.PI / 180; // 40
        public static final double coralPickUpSetPoint_Rad = 175 * Math.PI / 180; // 175

        public static final double maxVelocity = 2.4223698370699234;
        public static final double maxAcceleration = 0.0;

        public static final double momentOfInertia = 0.775; // from last years MOI
        public static final boolean simulateGravity = true;

        public enum ArmStates {
            MOVING_DOWN,
            MOVING_UP,
            CORAL_PICKUP,
            HAND_OFF,
            STOWED,
            STOPPED
        }
    }

    public class climbArmConstants {
        public static final int firstMotorId = 0;
        public static final int secondMotorId = 1;
        public static final int numMotors = 1;

        public static final int armLength_inches = 16;
        public static final double armLength_meters = armLength_inches / 39.3701;

        public static final int armMinAngle_degrees = -30;
        public static final int armMaxAngle_degrees = 200;
        public static final int armStartingAngle_degrees = 0;

        public static final double armMass_kg = 8.44;
        public static final double armGearing = 255;

        public static double kP = 15;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static double kG = 0.50738;
        public static final double kS = 0.0;
        public static final double kA = 0.0;
        public static final double kV = 0.0;

        public static final double stowSetPoint_rad = 0;
        public static final double PrepareForClimbSetPoint_Rad = Math.PI / 2; // 90
        public static final double climbSetPoint_rad = 20 * Math.PI / 180; // 20

        public static final double maxVelocity = 2.4223698370699234;
        public static final double maxAcceleration = 0.0;

        public static final double momentOfInertia = 0.775;
        public static final boolean simulateGravity = true;

        public enum ArmStates {
            MOVING_DOWN,
            MOVING_UP,
            PREPARE_FOR_CLIMB,
            CLIMB,
            STOWED,
            STOPPED
        }
    }
}
