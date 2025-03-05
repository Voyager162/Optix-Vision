package frc.robot.subsystems.arm.climb;

import frc.robot.Robot;
import frc.robot.utils.UtilityFunctions;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.climb.ClimbArmIO.ArmData;
import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.arm.climb.real.ClimbArmSparkMax;
import frc.robot.subsystems.arm.climb.sim.ClimbArmSim;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * Subsystem class for the climb arm
 *
 * @author Weston Gardner
 */

public class ClimbArm extends SubsystemBase {

	private ProfiledPIDController profile = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(
			ClimbArmConstants.maxVelocity.get(), ClimbArmConstants.maxAcceleration.get()));

	private ArmFeedforward feedforward = new ArmFeedforward(ClimbArmConstants.kS.get(), ClimbArmConstants.kG.get(),
			ClimbArmConstants.kV.get());

	private ClimbArmIO armIO;
	private ArmData data = new ArmData();
	private ClimbArmConstants.ArmStates state = ClimbArmConstants.ArmStates.STOPPED;

	private LoggedMechanism2d mechanism2d = new LoggedMechanism2d(60, 60);
	private LoggedMechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
	private LoggedMechanismLigament2d armLigament = armRoot.append(new LoggedMechanismLigament2d("ClimbArm", 24, 0));

	StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("ClimbArm Pose", Pose3d.struct).publish();

	public ClimbArm() {
		if (Robot.isSimulation()) {

			armIO = new ClimbArmSim();

		} else {
			armIO = new ClimbArmSparkMax();
		}

		feedforward = new ArmFeedforward(ClimbArmConstants.kS.get(), ClimbArmConstants.kG.get(),
				ClimbArmConstants.kV.get());
		profile = new ProfiledPIDController(ClimbArmConstants.kP.get(), ClimbArmConstants.kI.get(),
				ClimbArmConstants.kD.get(),
				new TrapezoidProfile.Constraints(ClimbArmConstants.maxVelocity.get(),
						ClimbArmConstants.maxAcceleration.get()));
		profile.reset(data.positionRad);

		setState(state);
	}

	/**
	 * @return the current arm state.
	 */
	public ClimbArmConstants.ArmStates getState() {
		return state;
	}

	/**
	 * @return the current arm position.
	 */
	public double getPositionRad() {
		return data.positionRad;
	}

	/**
	 * @return whether the arm is in a stable state.
	 */
	public boolean getIsStableState() {

		switch (state) {
			case STOWED:
				return UtilityFunctions.withinMargin(ClimbArmConstants.stateMarginOfError,
						ClimbArmConstants.stowSetPointRad.get(), data.positionRad);
			case CLIMB:
				return UtilityFunctions.withinMargin(ClimbArmConstants.stateMarginOfError,
						ClimbArmConstants.climbVoltage.get(),
						(data.backMotorAppliedVolts + data.frontMotorAppliedVolts) / 2);
			case STOPPED:
				return UtilityFunctions.withinMargin(ClimbArmConstants.stateMarginOfError, 0, data.velocityRadPerSec);
			default:
				return false;
		}
	}

	// SET FUNCTIONS

	/**
	 * method to set the voltage for the arm
	 * 
	 * @param volts
	 */
	public void setVoltage(double volts) {
		armIO.setVoltage(volts);
	}

	/**
	 * Sets the current state of the arm.
	 *
	 * @param state The new state for the arm.
	 */
	public void setState(ClimbArmConstants.ArmStates state) {
		this.state = (ClimbArmConstants.ArmStates) state;
		switch (this.state) {
			case STOPPED:
				stop();
				break;
			case STOWED:
				armIO.setVoltage(-1);
				break;
			case CLIMB:
			
				armIO.setVoltage(ClimbArmConstants.climbVoltage.get());
			default:
				stop();
				break;
		}
	}

	/**
	 * Sets the brake mode of the arm.
	 *
	 * @param setMode The new brake mode for the arm.
	 */
	public void setBrakeMode(boolean setMode) {
		armIO.setBrakeMode(setMode);
	}

	private Angle getPitch() {
		return Angle.ofBaseUnits(-data.positionRad + Units.degreesToRadians(0), Radians); // remove offset once climb
																							// arm code is fixed
	}

	private Pose3d getPose3d() {
		Pose3d pose = new Pose3d(-0.33, 0.18, 0.165,
				new Rotation3d(getPitch(), Angle.ofBaseUnits(0, Radians), Angle.ofBaseUnits(0, Radians)));
		return pose;
	}

	/**
	 * method to set the goal of the controller
	 * 
	 * @param setPoint
	 */
	public void setGoal(double setPoint) {
		profile.setGoal(setPoint);
	}

	/**
	 * stops the arm completely, for use in emergencies or on startup
	 */
	public void stop() {
		setVoltage(0);
	}

	/**
	 * Move the arm to the setpoint using the PID controller and feedforward.
	 * combines PID control and feedforward to move the arm to desired position.
	 */
	private void moveToGoal() {

		State firstState = profile.getSetpoint();

		double pidVoltage = profile.calculate(getPositionRad());
		State nextState = profile.getSetpoint();

		double ffVoltage = feedforward.calculateWithVelocities(getPositionRad(), firstState.velocity, nextState.velocity);

		double volts = ffVoltage + pidVoltage;
		armIO.setVoltage(volts);
	}

	// PERIODIC FUNCTIONS

	/** Runs the logic for the current arm state. */
	private void runState() {
		switch (state) {
			case CLIMB:
				armIO.setVoltage(ClimbArmConstants.climbVoltage.get());
				break;
			case STOPPED:
				stop();
				break;
			default:
				// moveToGoal();
				break;
		}
	}

	/** Logs data to Shuffleboard. */
	private void logData() {

		Logger.recordOutput("Arms/ClimbArm/currentCommand",
				this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
		Logger.recordOutput("Arms/ClimbArm/currentState", state.name());

		Logger.recordOutput("Arms/ClimbArm/position", data.positionRad);
		Logger.recordOutput("Arms/ClimbArm/velocity", data.velocityRadPerSec);

		Logger.recordOutput("Arms/ClimbArm/inputVolts", data.inputVolts);
		Logger.recordOutput("Arms/ClimbArm/frontMotor/appliedVolts", data.frontMotorAppliedVolts);
		Logger.recordOutput("Arms/ClimbArm/backMotor/appliedVolts", data.backMotorAppliedVolts);

		Logger.recordOutput("Arms/ClimbArm/frontMotor/currentAmps", data.frontMotorCurrentAmps);
		Logger.recordOutput("Arms/ClimbArm/backMotor/currentAmps", data.backMotorCurrentAmps);

		Logger.recordOutput("Arms/ClimbArm/frontMotor/temperature", data.frontMotorTempCelcius);
		Logger.recordOutput("Arms/ClimbArm/backMotor/temperature", data.backMotorTempCelcius);

		Logger.recordOutput("Arms/ClimbArm/climbArmMechanism", mechanism2d);

		armLigament.setAngle(Math.toDegrees(data.positionRad));
		publisher.set(getPose3d());
	}

	/**
	 * Periodic method for updating arm behavior.
	 */
	@Override
	public void periodic() {

		armIO.updateData(data);

		runState();

		logData();
	}
}
