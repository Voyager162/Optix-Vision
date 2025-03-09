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
	public static final double absoluteEncoderOffsetRad = 6.13 - Math.PI * 0.5;

	// arm specifications
	public static final int armLengthInches = 16;
	public static final double armLengthMeters = Units.inchesToMeters(armLengthInches);

	public static final int armMinAngleDegrees = -30;
	public static final int armMaxAngleDegrees = 200;
	public static final int armStartingAngleDegrees = 0;

	public static final double armMassKg = 8.44;
	public static final double armGearing = 125;

	public static final double momentOfInertia = 0.775;

	// control values
	public static LoggedTunableNumber kG = new LoggedTunableNumber("Arms/ClimbArm/kG", 0.0065);
	public static LoggedTunableNumber kP = new LoggedTunableNumber("Arms/ClimbArm/kP", 0.0);
	public static LoggedTunableNumber kI = new LoggedTunableNumber("Arms/ClimbArm/kP" + "/kI", 0);
	public static LoggedTunableNumber kD = new LoggedTunableNumber("Arms/ClimbArm/kD", 0);

	public static LoggedTunableNumber kS = new LoggedTunableNumber("Arms/ClimbArm/kS", 0.1115);
	public static LoggedTunableNumber kV = new LoggedTunableNumber("Arms/ClimbArm/kV", 2.439);
	public static LoggedTunableNumber kA = new LoggedTunableNumber("Arms/ClimbArm/kA", 0.447);
	public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Arms/ClimbArm/max velocity",
			4.6);
	public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
			"Arms/ClimbArm/max acceleration",
			18.4);

	// setpoints
	public static final LoggedTunableNumber stowSetPointRad = new LoggedTunableNumber("Arms/ClimbArm/stowSetPointRad",
			Math.PI / 2);
	public static final LoggedTunableNumber climbVoltage = new LoggedTunableNumber("Arms/ClimbArm/climbVoltage", 10.5);

	// extra
	public static final boolean simulateGravity = true;
	public static final double stateMarginOfError = 0.01;

	public enum ArmStates {
		CLIMB,
		STOWED,
		STOPPED
	}
}
