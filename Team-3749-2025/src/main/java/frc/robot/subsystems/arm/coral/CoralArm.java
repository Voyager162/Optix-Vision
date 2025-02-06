package frc.robot.subsystems.arm.coral;

import frc.robot.Robot;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.UtilityFunctions;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.arm.coral.CoralArmIO.ArmData;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.MotorData;
import static edu.wpi.first.units.Units.*;

import java.util.Map;

/**
 * Subsystem class for the coral arm
 *
 * @author Weston Gardner
 */
public class CoralArm extends SubsystemBase {

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

	private CoralArmIO armIO;
	private ArmData data = new ArmData();
	private CoralConstants.ArmStates state = CoralConstants.ArmStates.STOPPED;

	private ShuffleData<String> currentCommandLog = new ShuffleData<>(this.getName(), "current command", "None");
	private LoggedTunableNumber positionUnitsLog = new LoggedTunableNumber(this.getName() + "/position units", 0.0);
	private LoggedTunableNumber velocityUnitsLog = new LoggedTunableNumber(this.getName() + "/velocity units", 0.0);
	private LoggedTunableNumber inputVoltsLog = new LoggedTunableNumber(this.getName() + "/input volts", 0.0);
	private LoggedTunableNumber motorAppliedVoltsLog = new LoggedTunableNumber(this.getName() +
			"/motor applied volts", 0.0);
	private LoggedTunableNumber motorCurrentAmpsLog = new LoggedTunableNumber(this.getName() +
			"/motor current amps", 0.0);
	private LoggedTunableNumber motorTempCelciusLog = new LoggedTunableNumber(this.getName() +
			"/motor temp celcius", 0.0);
	private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

	private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
	private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
	private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Coral Arm", 24, 0));

    private SysIdTuner sysIdTuner;

	Map<String, MotorData> motorData = Map.of(
            "arm_motor", new MotorData(
                    data.appliedVolts,
                    data.positionUnits,
                    data.velocityUnits,
                    data.accelerationUnits));

	SysIdRoutine.Config config = new SysIdRoutine.Config(
            Volts.per(Seconds).of(1), // Voltage ramp rate
            Volts.of(4), // Max voltage
            Seconds.of(4) // Test duration
    );

	/**
	 * Constructor for the CoralArm subsystem. Determines if simulation or real
	 * hardware is used.
	 */
	public CoralArm() {

		// If the robot is in simulation, use the simulated I/O for the arm.
		if (Robot.isSimulation()) {
			armIO = new CoralArmSim();

		} else { 
			// If running on real hardware, use SparkMax motors for the arm.
			armIO = new CoralArmSparkMax(CoralConstants.motorID);
		}
		
		// Add the arm visualization to the SmartDashboard
		SmartDashboard.putData("Coral Arm Mechanism", mechanism2d);

        sysIdTuner = new SysIdTuner("coral arm", getConfig(), this, armIO::setVoltage, getMotorData());
    }

	public SysIdRoutine.Config getConfig(){
		return config;
	}

	public Map<String, MotorData> getMotorData(){
		return motorData;
	}

    public SysIdTuner getSysIdTuner(){
        return sysIdTuner;
    }


	// SET FUNCTIONS


	// Method to set the voltage for the arm
	public void setVoltage(double volts) {
		armIO.setVoltage(volts);
	}

	/**
	 * Sets the current state of the arm.
	 *
	 * @param state The new state for the arm.
	 */
	public void setState(CoralConstants.ArmStates state) {
		this.state = (CoralConstants.ArmStates) state;
		switch (this.state) {
			case STOPPED:
				stop();
				break;
			case STOWED:
				setGoal(CoralConstants.stowSetPoint_rad);
				break;
			case CORAL_PICKUP:
				setGoal(CoralConstants.coralPickUpSetPoint_rad);
			case HAND_OFF:
				setGoal(CoralConstants.handOffSetPoint_rad);
			default:
				stop();
				break;
		}
	}

	public void setGoal(double setPoint) {
		controller.setGoal(setPoint);
	}


	// GET FUNCTIONS

	/**
	 * @return The current arm state (e.g., STOPPED, STOWED, etc.)
	 */
	public CoralConstants.ArmStates getState() {
		return state;
	}

	/**
	 * @return The current position of the arm in radians.
	 */
	public double getPositionRad() {
		return data.positionUnits; // Return the arm's current position.
	}

	/**
	 * @return whether the arm is in a stable state.
	 */
	public boolean getIsStableState() {

		switch (state) {
			case STOWED:
				return UtilityFunctions.withinMargin(0.001, CoralConstants.stowSetPoint_rad, data.positionUnits);
			case HAND_OFF:
				return UtilityFunctions.withinMargin(0.001, CoralConstants.handOffSetPoint_rad, data.positionUnits);
			case CORAL_PICKUP:
				return UtilityFunctions.withinMargin(0.001, CoralConstants.coralPickUpSetPoint_rad, data.positionUnits);
			case STOPPED:
				return UtilityFunctions.withinMargin(0.001, 0, data.velocityUnits);
			default:
				return false;
		}
	}


	// UTILITY FUNCTIONS

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
		State firstState = controller.getSetpoint();

		// Calculate PID voltage based on the current position
		double pidVoltage = controller.calculate(getPositionRad());

		State nextState = controller.getSetpoint();

		// Calculate feedforward voltage
		double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

		// Set the voltage for the arm motor (combine PID and feedforward)
		armIO.setVoltage(pidVoltage + ffVoltage);
	}


	// PERIODIC FUNCTIONS



	/** Runs the logic for the current arm state. */
	private void runState() {
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
		currentCommandLog.set(
				this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
		positionUnitsLog.set(data.positionUnits);
		velocityUnitsLog.set(data.velocityUnits);
		inputVoltsLog.set(data.inputVolts);
		motorAppliedVoltsLog.set(data.motorAppliedVolts);
		motorCurrentAmpsLog.set(data.motorCurrentAmps);
		motorTempCelciusLog.set(data.motorTempCelcius);

		armLigament.setAngle(Math.toDegrees(data.positionUnits));

		stateLog.set(state.name());
	}

	/** Periodic method for updating arm behavior. */
	@Override
	public void periodic() {

		armIO.updateData(data);

		runState();

		logData();

        getMotorData().get("arm_motor").position = data.positionUnits;
        getMotorData().get("arm_motor").acceleration = data.accelerationUnits;
        getMotorData().get("arm_motor").velocity = data.velocityUnits;
        getMotorData().get("arm_motor").appliedVolts = data.appliedVolts;
	}
}
