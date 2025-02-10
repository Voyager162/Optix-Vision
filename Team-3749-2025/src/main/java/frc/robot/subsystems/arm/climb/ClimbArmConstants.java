package frc.robot.subsystems.arm.climb;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.LoggedTunableNumber;

/**
 * Constants file for the climb arm subsystem
 *
 * @author Weston Gardner
 */
public class ClimbArmConstants {

	// motors
	public static final int frontMotorId = 17;
	public static final int backMotorId = 18;
	public static final int numMotors = 2;

	// arm specifications
	public static final int armLength_inches = 16;
	public static final double armLength_meters = Units.inchesToMeters(armLength_inches);

	public static final int armMinAngle_degrees = -30;
	public static final int armMaxAngle_degrees = 200;
	public static final int armStartingAngle_degrees = 0;

	public static final double armMass_kg = 8.44;
	public static final double armGearing = 255;


	public static final double momentOfInertia = 0.775;

	// control values
	public static LoggedTunableNumber kG = new LoggedTunableNumber("/subsystems/arms/climbArm/kG", 0.5);
	public static LoggedTunableNumber kP = new LoggedTunableNumber("/subsystems/arms/climbArm/kP", 15);
	public static LoggedTunableNumber kI = new LoggedTunableNumber("/subsystems/arms/climbArm/kP" + "/kI", 0);
	public static LoggedTunableNumber kD = new LoggedTunableNumber("/subsystems/arms/climbArm/kD", 0);
	public static LoggedTunableNumber kS = new LoggedTunableNumber("/subsystems/arms/climbArm/kS", 0);
	public static LoggedTunableNumber kV = new LoggedTunableNumber("/subsystems/arms/climbArm/kV", 0);
	public static LoggedTunableNumber kA = new LoggedTunableNumber("/subsystems/arms/climbArm/kA", 0);
	public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("/subsystems/arms/climbArm/max velocity",
			0);
	public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber("/subsystems/arms/climbArm/max acceleration",
			0);

	// setpoints
	public static final double stowSetPoint_rad = 0;
	public static final double PrepareForClimbSetPoint_rad = Math.PI / 2; // 90
	public static final double climbSetPoint_rad = 20 * Math.PI / 180; // 20

	// extra
	public static final boolean simulateGravity = true;
	public static final double stateMarginOfError = 0.01;

	public enum ArmStates {
		PREPARE_FOR_CLIMB,
		CLIMB,
		STOWED,
		STOPPED
	}
}
