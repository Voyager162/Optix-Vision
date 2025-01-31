package frc.robot.subsystems.arm.coral;

import edu.wpi.first.math.util.Units;

/**
 * Constants file for the climb arm subsystem
 *
 * @author Weston Gardner
 */
public class CoralConstants {

	// motor specifications
	public static final int firstMotorID = 0;
	public static final int secondMotorID = 0;
	public static final int numMotors = 1;

	// arm specifications
	public static final double armLength_inches = 17.796;
	public static final double armLength_meters = Units.inchesToMeters(armLength_inches);

	public static final int armMinAngle_degrees = 10;
	public static final int armMaxAngle_degrees = 180;
	public static final int armStartingAngle_degrees = 90;

	public static final double armMass_kg = 1.132;
	public static final double armGearing = 40;

	public static final double maxVelocity = 5;
	public static final double maxAcceleration = 4;

	public static final double momentOfInertia = 0.775;

	// control values
	public static final double kP = 2;
	public static final double kI = 0.0;
	public static final double kD = 0.0;

	public static final double kG = 1.454056;
	public static final double kS = 0.0;
	public static final double kA = 0.0;
	public static final double kV = 1.5;

	// set points
	public static final double stowSetPoint_rad = 70 * Math.PI / 180;
	public static final double handOffSetPoint_rad = 40 * Math.PI / 180;
	public static final double coralPickUpSetPoint_rad = 175 * Math.PI / 180;
	
	// extra
	public static final boolean simulateGravity = true;

	public enum ArmStates {
		CORAL_PICKUP,
		HAND_OFF,
		STOWED,
		STOPPED
	}
}
