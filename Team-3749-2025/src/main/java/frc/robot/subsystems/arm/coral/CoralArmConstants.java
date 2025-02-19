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
	public static final double absoluteEncoderOffsetRad = Units.degreesToRadians(39.32905);

	public static final double armLengthIn = 17.796; // from cad max length of arm
	public static final double armLengthMeters = Units.inchesToMeters(armLengthIn);

	public static final int armMinAngleDegrees = 10;
	public static final int armMaxAngleDegrees = 180;
	public static final int armStartingAngleDegrees = 90;

	public static final double armMassKg = 1.132; // from cad, I highlighted all of the components on the arm and used
													// the mass feature
	public static final double armGearing = 45;

	// control values
	public static LoggedTunableNumber kG = new LoggedTunableNumber("/subsystems/arms/coralArm/kG", 0.39);
	public static LoggedTunableNumber kP = new LoggedTunableNumber("/subsystems/arms/coralArm/kP", 2.75);
	public static LoggedTunableNumber kI = new LoggedTunableNumber("/subsystems/arms/coralArm/kP" + "/kI", 0);
	public static LoggedTunableNumber kD = new LoggedTunableNumber("/subsystems/arms/coralArm/kD", 0);
	public static LoggedTunableNumber kS = new LoggedTunableNumber("/subsystems/arms/coralArm/kS", 0.16);
	public static LoggedTunableNumber kV = new LoggedTunableNumber("/subsystems/arms/coralArm/kV", 0.61);
	public static LoggedTunableNumber kA = new LoggedTunableNumber("/subsystems/arms/coralArm/kA", 0);
	public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("/subsystems/arms/coralArm/max velocity",
			6);
	public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
			"/subsystems/arms/coralArm/max acceleration",
			12);

	public static final double stowSetPointRad = 2.37; // 70, 1.2217304764
	public static final double handOffSetPointRad = 2.37; // 40
	public static final double coralPickUpSetPointRad = -Units.degreesToRadians(38.5); // 175, 3.05432619099
	public static final double l1SetPointRad = Math.PI/4;

	public static final double momentOfInertia = 0.775;

	public static final boolean simulateGravity = true;
	public static final double stateMarginOfError = 0.01;

	public enum ArmStates {
		CORAL_PICKUP,
		L1,
		SOURCE,
		HAND_OFF,
		STOWED,
		STOPPED
	}
}
