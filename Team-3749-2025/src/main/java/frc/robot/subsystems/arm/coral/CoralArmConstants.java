package frc.robot.subsystems.arm.coral;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.LoggedTunableNumber;

/**
 * Constants file for the climb arm subsystem
 *
 * @author Weston Gardner
 */
public class CoralArmConstants {

	// motor specifications
	public static final int motorID = 16;
	public static final int numMotors = 1;

	public static final double armLength_inches = 17.796; // from cad max length of arm
	public static final double armLength_meters = Units.inchesToMeters(armLength_inches);

	public static final int armMinAngle_degrees = 10;
	public static final int armMaxAngle_degrees = 180;
	public static final int armStartingAngle_degrees = 90;

	public static final double armMass_kg = 1.132; // from cad, I highlighted all of the components on the arm and used
													// the mass feature
	public static final double armGearing = 40;

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

	public static final double stowSetPoint_rad = 70 * Math.PI / 180; // 70, 1.2217304764
	public static final double handOffSetPoint_rad = 40 * Math.PI / 180; // 40
	public static final double coralPickUpSetPoint_rad = 175 * Math.PI / 180; // 175, 3.05432619099

	public static final double momentOfInertia = 0.775; // from last years MOI

	public static final boolean simulateGravity = true;
	public static final double stateMarginOfError = 0.01;

	public enum ArmStates {
		CORAL_PICKUP,
		HAND_OFF,
		STOWED,
		STOPPED
	}
}
