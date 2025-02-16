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
	public static final double absoluteEncoderOffsetRad = 0.595;

	public static final double armLength_inches = 17.796; // from cad max length of arm
	public static final double armLength_meters = Units.inchesToMeters(armLength_inches);

	public static final int armMinAngle_degrees = 10;
	public static final int armMaxAngle_degrees = 180;
	public static final int armStartingAngle_degrees = 90;

	public static final double armMass_kg = 1.132; // from cad, I highlighted all of the components on the arm and used
													// the mass feature
	public static final double armGearing = 45;

	// control values
	public static LoggedTunableNumber kG = new LoggedTunableNumber("/subsystems/arms/coralArm/kG", 0.365);
	public static LoggedTunableNumber kP = new LoggedTunableNumber("/subsystems/arms/coralArm/kP", 0);
	public static LoggedTunableNumber kI = new LoggedTunableNumber("/subsystems/arms/coralArm/kP" + "/kI", 0);
	public static LoggedTunableNumber kD = new LoggedTunableNumber("/subsystems/arms/coralArm/kD", 0);
	public static LoggedTunableNumber kS = new LoggedTunableNumber("/subsystems/arms/coralArm/kS", 0.135);
	public static LoggedTunableNumber kV = new LoggedTunableNumber("/subsystems/arms/coralArm/kV", 0.85);
	public static LoggedTunableNumber kA = new LoggedTunableNumber("/subsystems/arms/coralArm/kA", 0.4);
	public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("/subsystems/arms/coralArm/max velocity",
			6);
	public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber("/subsystems/arms/coralArm/max acceleration",
			10);

	public static final double stowSetPoint_rad = 2.51; // 70, 1.2217304764
	public static final double handOffSetPoint_rad = 2.51; // 40
	public static final double coralPickUpSetPoint_rad = -0.57; // 175, 3.05432619099

	public static final double momentOfInertia = 0.775; // from last years MOI

	public static final boolean simulateGravity = true;
	public static final double stateMarginOfError = 0.01;

    public enum ArmStates {
        MOVING_DOWN,
        MOVING_UP,
        CORAL_PICKUP,
        L1,
        SOURCE,
        HAND_OFF,
        STOWED,
        STOPPED
    }
}
