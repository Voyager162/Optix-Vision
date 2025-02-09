package frc.robot.subsystems.arm.coral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's simulation.
 * This class simulates behaviour
 * @author Weston Gardner
 */
public class CoralArmSim implements CoralArmIO {

	private SingleJointedArmSim armSim; // Simulated arm system

	/**
	 * Initializes arm simulation
	 * 
	 * @param numMotors           Number of motors driving the arm
	 * @param gearing             Gear ratio of the motor system
	 * @param momentOfInertia     Rotational inertia of the arm
	 * @param length_meters       Length of the arm in meters
	 * @param minAngle_degrees    Minimum angle limit in degrees
	 * @param maxAngle_degrees    Maximum angle limit in degrees
	 * @param simulateGravity     Whether to simulate gravity effects
	 * @param startingAngle_Degrees Initial angle of the arm in degrees
	 */
	public CoralArmSim(
			int numMotors,
			double gearing,
			double momentOfInertia,
			double length_meters,
			double minAngle_degrees,
			double maxAngle_degrees,
			boolean simulateGravity,
			double startingAngle_Degrees) {

		System.out.println("[Init] Creating ArmSim");

		// Initialize arm simulation with motor parameters and physics properties
		armSim = new SingleJointedArmSim(
				DCMotor.getNEO(numMotors), // Creates a simulated NEO motor with the specified count
				gearing,                   // Gear ratio applied to the motor output
				momentOfInertia,           // Moment of inertia for realistic motion simulation
				length_meters,             // Length of the arm (affects torque calculations)
				minAngle_degrees * Math.PI / 180, // Convert min angle from degrees to radians
				maxAngle_degrees * Math.PI / 180, // Convert max angle from degrees to radians
				simulateGravity,           // Enable or disable gravity effects
				startingAngle_Degrees * Math.PI / 180); // Convert starting angle to radians
	}

	// Stores the last commanded voltage input to the motor
	private double inputVolts = 0;
	private double previousVelocity = 0; // Stores the previous velocity for acceleration calculations
	private double velocity = 0; // Stores the current velocity of the arm

	/**
	 * Updates the set of loggable inputs for the simulation.
	 * This method runs every loop iteration to update the arm's state.
	 *
	 * @param data The ArmData object that stores position, velocity, acceleration, and other details.
	 */
	@Override
	public void updateData(ArmData data) {
		// Advance the simulation by 20 milliseconds (one loop iteration)
		armSim.update(0.02);

		// Store previous velocity for acceleration calculation
		previousVelocity = velocity;
		velocity = armSim.getVelocityRadPerSec(); // Get current angular velocity in radians/sec

		// Update the data object with simulation values
		data.positionUnits = armSim.getAngleRads(); // Arm's current position in radians
		data.velocityUnits = velocity; // Arm's current velocity in radians/sec
		data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec; // Compute acceleration

		// Log the motor input voltage and actual applied voltage
		data.inputVolts = inputVolts;
		data.motorAppliedVolts = inputVolts;

		// Get simulated motor current draw
		data.motorCurrentAmps = armSim.getCurrentDrawAmps();

		// Simulation does not track motor temperature, so set it to 0
		data.motorTempCelcius = 0;
	}

	/**
	 * Runs the motor at the specified voltage.
	 * Applies a voltage command to the arm's motor while clamping it to safe limits.
	 *
	 * @param volts The desired motor voltage, should be in the range of -12V to 12V.
	 */
	@Override
	public void setVoltage(double volts) {
		// Apply a small deadband to prevent small voltage inputs from affecting the motor
		inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);

		// Clamp the voltage to ensure it stays within the valid range of [-12V, 12V]
		inputVolts = MathUtil.clamp(volts, -12, 12);

		// Apply the voltage to the simulated arm motor
		armSim.setInputVoltage(inputVolts);
	}
}
