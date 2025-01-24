package frc.robot.subsystems.arm.climb;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.real.ArmSparkMax;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Subsystem class for controlling the climbing arm.
 * Handles the arm's states and manages control over the arm motors.
 */
public class ClimbArm extends Arm {

	// Current state of the arm (e.g., moving up, moving down, stopped, etc.)
	private ClimbConstants.ArmStates state = ClimbConstants.ArmStates.STOPPED;

	// PID controller for arm movement
	private ProfiledPIDController controller = new ProfiledPIDController(
			ClimbConstants.kP,
			ClimbConstants.kI,
			ClimbConstants.kD,
			new TrapezoidProfile.Constraints(
					ClimbConstants.maxVelocity,
					ClimbConstants.maxAcceleration));

	private ArmFeedforward feedforward = new ArmFeedforward(
			ClimbConstants.kG,
			ClimbConstants.kS,
			ClimbConstants.kA,
			ClimbConstants.kV);

	// Shuffleboard data for state logging
	private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

	// Mechanism visualization for logging the arm's position on Shuffleboard
	private LoggedMechanism2d mechanism2d = new LoggedMechanism2d(60, 60);
	private LoggedMechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
	private LoggedMechanismLigament2d armLigament = armRoot.append(new LoggedMechanismLigament2d("Climb Arm", 24, 0));

	// Constructor to initialize arm's hardware or simulation
	public ClimbArm() {
		if (Robot.isSimulation()) {
			armIO = new ArmSim(
					ClimbConstants.armGearing,
					ClimbConstants.momentOfInertia,
					ClimbConstants.armLength_meters,
					ClimbConstants.armMinAngle_degrees,
					ClimbConstants.armMaxAngle_degrees,
					ClimbConstants.simulateGravity,
					ClimbConstants.armStartingAngle_degrees);
		} else {
			armIO = new ArmSparkMax(ClimbConstants.firstMotorId, ClimbConstants.secondMotorId);
		}
		SmartDashboard.putData("Climb Arm Mechanism", mechanism2d);
	}

	/**
	 * @return the current arm state.
	 */
	public ClimbConstants.ArmStates getState() {
		return state;
	}

	@Override
	public void stop() {
		setState(ClimbConstants.ArmStates.STOPPED);
		armIO.setVoltage(0);
	}

	/**
	 * @return whether the arm is in a stable state.
	 */
	public boolean getIsStableState() {
		switch (state) {
			case STOWED:
				return data.positionUnits == ClimbConstants.stowSetPoint_rad;
			case PREPARE_FOR_CLIMB:
				return data.positionUnits == ClimbConstants.PrepareForClimbSetPoint_rad;
			case CLIMB:
				return data.positionUnits == ClimbConstants.climbSetPoint_rad;
			case MOVING_DOWN:
				return data.velocityUnits < 0;
			case MOVING_UP:
				return data.velocityUnits > 0;
			case STOPPED:
				return UtilityFunctions.withinMargin(0.001, 0, data.velocityUnits);
			default:
				return false;
		}
	}

	/**
	 * Sets the current state of the arm.
	 *
	 * @param state The new state for the arm.
	 */
	@Override
	public void setState(Enum<?> state) {
		this.state = (ClimbConstants.ArmStates) state;
		switch (this.state) {
			case STOPPED:
				runStateStop();
				break;
			case STOWED:
				setGoal(ClimbConstants.stowSetPoint_rad);
				break;
			case CLIMB:
				setGoal(ClimbConstants.climbSetPoint_rad);
			case PREPARE_FOR_CLIMB:
				setGoal(ClimbConstants.PrepareForClimbSetPoint_rad);
			case MOVING_DOWN:
				setVoltage(ClimbConstants.staticMovementVoltage + calculateKGFeedForward());
				break;
			case MOVING_UP:
				setVoltage(-ClimbConstants.staticMovementVoltage + calculateKGFeedForward());
				break;
			default:
				stop();
				break;
		}
	}

	private void runStateStop() {
		stop();
	}

	public void setGoal(double setPoint) {
		controller.setGoal(setPoint);
	}

	/** Runs the logic for the current arm state. */
	private void runState() {
		switch (state) {
			case STOPPED:
				runStateStop();
				break;
			case MOVING_DOWN:
				setVoltage(-1 + calculateKGFeedForward());
				break;
			case MOVING_UP:
				setVoltage(1 + calculateKGFeedForward());
				break;
			default:
				moveToGoal();
				break;
		}
	}

	private double calculateKGFeedForward() {
		// Calculate feedforward based on the arm's position
		double feedForward = ClimbConstants.kG * Math.cos(data.positionUnits);
		return feedForward;
	}

	/**
	 * Move the arm to the setpoint using the PID controller and feedforward.
	 * combines PID control and feedforward to move the arm to desired position.
	 */
	private void moveToGoal() {
		// Get setpoint from the PID controller
		State firstState = controller.getSetpoint();

		// Calculate PID voltage based on the current position
		double pidVoltage = controller.calculate(getPositionRad());

		State nextState = controller.getSetpoint();

		// Calculate feedforward voltage
		double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

		// Set the voltage for the arm motor (combine PID and feedforward)
		armIO.setVoltage(pidVoltage + ffVoltage);
	}

	/** Logs data to Shuffleboard. */
	private void logData() {
		currentCommandLog.set(
				this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
		positionUnitsLog.set(data.positionUnits);
		velocityUnitsLog.set(data.velocityUnits);
		inputVoltsLog.set(data.inputVolts);
		firstMotorAppliedVoltsLog.set(data.firstMotorAppliedVolts);
		secondMotorAppliedVoltsLog.set(data.secondMotorAppliedVolts);
		firstMotorCurrentAmpsLog.set(data.firstMotorCurrentAmps);
		secondMotorCurrentAmpsLog.set(data.secondMotorCurrentAmps);
		firstMotorTempCelciusLog.set(data.firstMotorTempCelcius);
		secondMotorTempCelciusLog.set(data.secondMotorTempCelcius);

		armLigament.setAngle(Math.toDegrees(data.positionUnits));

		stateLog.set(state.name());

		Logger.recordOutput(this.getName(), mechanism2d);
	}

	/** Periodic method for updating arm behavior. */
	@Override
	public void periodic() {
		// Update data from the arm I/O (motor controllers or simulator)
		armIO.updateData(data);

		// Log current data to Shuffleboard
		logData();

		// Run the current state logic
		runState();
	}
}
