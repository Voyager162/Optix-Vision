package frc.robot.subsystems.arm.climb.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.climb.ClimbArmIO;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.utils.MiscConstants.SimConstants;
import frc.robot.utils.UtilityFunctions;

/**
 * IO implementation for an arm subsystem's simulation
 *
 * @author Weston Gardner
 */
public class ClimbArmSim implements ClimbArmIO {

	private SingleJointedArmSim armSim;

	private double inputVolts = 0;
	private double previousVelocity = 0;
	private double velocity = 0;

	/**
	 * creates a new arm simulation motor form the single jointed arm class
	 */
	public ClimbArmSim() {

		System.out.println("[Init] Creating ArmSim");

		armSim = new SingleJointedArmSim(
				DCMotor.getNEO(ClimbArmConstants.numMotors),
				ClimbArmConstants.armGearing,
				ClimbArmConstants.momentOfInertia,
				ClimbArmConstants.armLengthMeters,
				ClimbArmConstants.armMinAngleDegrees * Math.PI / 180,
				ClimbArmConstants.armMaxAngleDegrees * Math.PI / 180,
				ClimbArmConstants.simulateGravity,
				ClimbArmConstants.armStartingAngleDegrees * Math.PI / 180);
	}

	/**
	 * Updates the set of loggable inputs for the sim.
	 * @param data
	 */
	@Override
	public void updateData(ArmData data) {
		armSim.update(0.02);
		previousVelocity = velocity;
		velocity = armSim.getVelocityRadPerSec();
		data.positionRad = armSim.getAngleRads();
		data.velocityRadPerSec = velocity;
		data.accelerationRadsPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;

		data.inputVolts = inputVolts;
		data.frontMotorAppliedVolts = inputVolts;
		data.backMotorAppliedVolts = inputVolts;
		data.frontMotorCurrentAmps = armSim.getCurrentDrawAmps();
		data.backMotorCurrentAmps = data.frontMotorCurrentAmps;

		// Sim has no temp
		data.frontMotorTempCelcius = 0;
		data.backMotorTempCelcius = 0;
	}

	/**
	 * Run the motor at the specified voltage.
	 *
	 * @param volts
	 */
	@Override
	public void setVoltage(double volts) {
		inputVolts = UtilityFunctions.applyDeadband(inputVolts, 0.05);
		inputVolts = MathUtil.clamp(volts, -12, 12);
		armSim.setInputVoltage(inputVolts);
	}
}
