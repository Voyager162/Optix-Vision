package frc.robot.subsystems.arm.coral;

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
 * Subsystem class for the arm
 *
 * @author Weston Gardner
 */
public class CoralArm extends Arm {

	private CoralConstants.ArmStates state = CoralConstants.ArmStates.STOPPED;

	private ProfiledPIDController controller = new ProfiledPIDController(
			CoralConstants.kP,
			CoralConstants.kI,
			CoralConstants.kD,
			new TrapezoidProfile.Constraints(
					CoralConstants.maxVelocity,
					CoralConstants.maxAcceleration));

	private ArmFeedforward feedforward = new ArmFeedforward(
			CoralConstants.kS,
			CoralConstants.kG,
			CoralConstants.kV,
			CoralConstants.kA);

	private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

	private LoggedMechanism2d mechanism2d = new LoggedMechanism2d(60, 60);
	private LoggedMechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
	private LoggedMechanismLigament2d armLigament = armRoot.append(new LoggedMechanismLigament2d("Coral Arm", 24, 0));

	/**
	 * Constructor for the CoralArm subsystem. Determines if simulation or real
	 * hardware is used.
	 */
	public CoralArm() {
		if (Robot.isSimulation()) {

			armIO = new ArmSim(
					CoralConstants.armGearing,
					CoralConstants.momentOfInertia,
					CoralConstants.armLength_meters,
					CoralConstants.armMinAngle_degrees,
					CoralConstants.armMaxAngle_degrees,
					CoralConstants.simulateGravity,
					CoralConstants.armStartingAngle_degrees);

		} else {
			armIO = new ArmSparkMax(CoralConstants.firstMotorID, CoralConstants.secondMotorID);
		}
		SmartDashboard.putData("Coral Arm Mechanism", mechanism2d);
	}

	/**
	 * @return the current arm state.
	 */
	public CoralConstants.ArmStates getState() {
		return state;
	}

	@Override
	public void stop() {
		setState(CoralConstants.ArmStates.STOPPED);
	}

	/**
	 * @return whether the arm is in a stable state.
	 */
	public boolean getIsStableState() {

		switch (state) {
			case STOWED:
				return data.positionUnits == CoralConstants.stowSetPoint_rad;
			case HAND_OFF:
				return data.positionUnits == CoralConstants.handOffSetPoint_rad;
			case CORAL_PICKUP:
				return data.positionUnits == CoralConstants.coralPickUpSetPoint_rad;
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
		this.state = (CoralConstants.ArmStates) state;
		switch (this.state) {
			case STOPPED:
				setVoltage(calculateKGFeedForward());
				break;
			case STOWED:
				setGoal(CoralConstants.stowSetPoint_rad);
				break;
			case CORAL_PICKUP:
				setGoal(CoralConstants.coralPickUpSetPoint_rad);
			case HAND_OFF:
				setGoal(CoralConstants.handOffSetPoint_rad);
			case MOVING_DOWN:
				setVoltage(CoralConstants.staticMovementVoltage + calculateKGFeedForward());
				break;
			case MOVING_UP:
				setVoltage(-CoralConstants.staticMovementVoltage + calculateKGFeedForward());
				break;
			default:
				setVoltage(calculateKGFeedForward());
				break;
		}
	}

	public void setGoal(double setPoint) {
		controller.setGoal(setPoint);
	}

	/** Runs the logic for the current arm state. */
	private void runState() {
		switch (state) {
			case STOPPED:
				stop();
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
		double feedForward = CoralConstants.kG * Math.cos(data.positionUnits);
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
		Logger.recordOutput("coral arm position", data.positionUnits);
	}

	/** Periodic method for updating arm behavior. */
	@Override
	public void periodic() {

		armIO.updateData(data);

		logData();

		runState();
	}
}
