package frc.robot.subsystems.arm.climb;

import frc.robot.Robot;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.SysIdTuner.Type;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.arm.climb.ClimbArmIO.ArmData;
import frc.robot.utils.MotorData;
import static edu.wpi.first.units.Units.*;

import java.util.Map;
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

	private ProfiledPIDController profile;
	private ArmFeedforward feedforward;
	private ClimbArmIO armIO;
	private ArmData data = new ArmData();
	private ClimbArmConstants.ArmStates state = ClimbArmConstants.ArmStates.STOPPED;

	private LoggedMechanism2d mechanism2d = new LoggedMechanism2d(60, 60);
	private LoggedMechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
	private LoggedMechanismLigament2d armLigament = armRoot.append(new LoggedMechanismLigament2d("Climb Arm", 24, 0));

	StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("ClimbArm Pose", Pose3d.struct).publish();
	/**
	 * Constructor for the CoralArm subsystem. Determines if simulation or real
	 * hardware is used.
	 */

	private SysIdTuner sysIdTuner;

	SysIdRoutine.Config config = new SysIdRoutine.Config(
			Volts.per(Seconds).of(1), // Voltage ramp rate
			Volts.of(12), // Max voltage
			Seconds.of(12) // Test duration
	);

	Map<String, MotorData> motorData = Map.of(
			"arm_motor", new MotorData(
					(data.frontMotorAppliedVolts + data.backMotorAppliedVolts) / 2,
					data.positionRad,
					data.velocityRadPerSec,
					data.accelerationUnits));

	public ClimbArm() {
		if (Robot.isSimulation()) {

			armIO = new ClimbArmSim();

		} else {
			armIO = new ClimbArmSparkMax();
		}

		sysIdTuner = new SysIdTuner("climb arm", getConfig(), this, armIO::setVoltage, getMotorData(), Type.ROTATIONAL);
	}

	public Map<String, MotorData> getMotorData() {
		return motorData;
	}

	public SysIdRoutine.Config getConfig() {
		return config;
	}

	public SysIdTuner getSysIdTuner() {
		System.out.println(sysIdTuner);
		return sysIdTuner;
	}

	// GET FUNCTIONS

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
						ClimbArmConstants.stowSetPoint_rad, data.positionRad);
			case PREPARE_FOR_CLIMB:
				return UtilityFunctions.withinMargin(ClimbArmConstants.stateMarginOfError,
						ClimbArmConstants.PrepareForClimbSetPoint_rad,
						data.positionRad);
			case CLIMB:
				return UtilityFunctions.withinMargin(ClimbArmConstants.stateMarginOfError,
						ClimbArmConstants.climbSetPoint_rad, data.positionRad);
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
		System.out.println(volts);
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
				setGoal(ClimbArmConstants.stowSetPoint_rad);
				break;
			case PREPARE_FOR_CLIMB:
				setGoal(ClimbArmConstants.PrepareForClimbSetPoint_rad);
			case CLIMB:
				setGoal(ClimbArmConstants.climbSetPoint_rad);
			default:
				stop();
				break;
		}
	}

	public void setBrakeMode(boolean setMode) {
		armIO.setBrakeMode(setMode);
	}

	private Angle getPitch() {
		return Angle.ofBaseUnits(-data.positionRad + Units.degreesToRadians(0), Radians); // remove offset once climb
																							// arm code is fixed
	}

	private Pose3d getPose3d() {
		//
		Pose3d pose = new Pose3d(0, 0.18, 0.165,
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
		// Get setpoint from the PID controller
		State firstState = profile.getSetpoint();

		// Calculate PID voltage based on the current position
		profile.calculate(getPositionRad());

		State nextState = profile.getSetpoint();

		// Calculate feedforward voltage
		double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

		// Set the voltage for the arm motor (combine PID and feedforward)
		armIO.setPosition(firstState.position, ffVoltage);
	}

	// PERIODIC FUNCTIONS

	/** Runs the logic for the current arm state. */
	public void runState() {
		switch (state) {
			case STOPPED:
				stop();
				break;
			default:
				moveToGoal();
				break;
		}
	}

	/** Logs data to Shuffleboard. */
	private void logData() {
		Logger.recordOutput("subsystems/arms/climbArm/Current Command",
				this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
		Logger.recordOutput("subsystems/arms/climbArm/position", data.positionRad);
		Logger.recordOutput("subsystems/arms/climbArm/velocity", data.velocityRadPerSec);
		Logger.recordOutput("subsystems/arms/climbArm/input volts", data.inputVolts);
		Logger.recordOutput("subsystems/arms/climbArm/frontMotor/applied volts", data.frontMotorAppliedVolts);
		Logger.recordOutput("subsystems/arms/climbArm/backMotor/applied volts", data.backMotorAppliedVolts);
		Logger.recordOutput("subsystems/arms/climbArm/frontMotor/current amps", data.frontMotorCurrentAmps);
		Logger.recordOutput("subsystems/arms/climbArm/backMotor/current amps", data.backMotorCurrentAmps);
		Logger.recordOutput("subsystems/arms/climbArm/frontMotor/temperature", data.frontMotorTempCelcius);
		Logger.recordOutput("subsystems/arms/climbArm/backMotor/temperature", data.backMotorTempCelcius);

		armLigament.setAngle(Math.toDegrees(data.positionRad));

		Logger.recordOutput("subsystems/arms/climbArm/current state", state.name());

		publisher.set(getPose3d());

		profile = new ProfiledPIDController(ClimbArmConstants.kP.get(), ClimbArmConstants.kI.get(),
				ClimbArmConstants.kD.get(),
				new TrapezoidProfile.Constraints(ClimbArmConstants.maxVelocity.get(),
						ClimbArmConstants.maxAcceleration.get()));
		feedforward = new ArmFeedforward(ClimbArmConstants.kS.get(), ClimbArmConstants.kG.get(),
				ClimbArmConstants.kV.get());

		Logger.recordOutput("subsystems/arms/climbArm/Climb Arm Mechanism", mechanism2d);
	}

	/**
	 * Periodic method for updating arm behavior.
	 */
	@Override
	public void periodic() {

		armIO.updateData(data);

		// runState();

		logData();

		motorData.get("arm_motor").position = data.positionRad;
		motorData.get("arm_motor").acceleration = data.accelerationUnits;
		motorData.get("arm_motor").velocity = data.velocityRadPerSec;
		motorData.get("arm_motor").appliedVolts = (data.frontMotorAppliedVolts + data.backMotorAppliedVolts) / 2;
	}
}
