package frc.robot.subsystems.arm.coral;

import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.real.ArmSparkMax;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem class for the arm
 * 
 * @author Weston Gardner
 */

public class CoralArm extends Arm {

    private CoralConstants.ArmStates state = CoralConstants.ArmStates.STOPPED;

    private PIDController controller = new PIDController(
            CoralConstants.kP,
            CoralConstants.kI,
            CoralConstants.kD);

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Coral Arm", 24, 0));


    /**
     * Constructor for the AlgaeArm subsystem.
     * Determines if simulation or real hardware is used.
     */
    public CoralArm() {
        if (Robot.isSimulation()) {

            armIO = new ArmSim(
                CoralConstants.numMotors,
                CoralConstants.armGearing,
                CoralConstants.momentOfInertia,
                CoralConstants.armLength_meters,
                CoralConstants.armMinAngle_degrees,
                CoralConstants.armMaxAngle_degrees,
                CoralConstants.simulateGravity,
                CoralConstants.armStartingAngle_degrees);

        } else {
            armIO = new ArmSparkMax(CoralConstants.motorId);
        }
        SmartDashboard.putData("Coral Arm Mechanism", mechanism2d);
    }

    /**
     * @return the current arm state.
     */
    public CoralConstants.ArmStates getState() {
        return state;
    }

    /**
     * sets the state to moving up
     */
    @Override
    public void moveUp() {
        setState(CoralConstants.ArmStates.MOVING_UP);
    }

    /**
     * sets the state to moving down
     */
    @Override
    public void moveDown() {
        setState(CoralConstants.ArmStates.MOVING_DOWN);
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
                return data.positionUnits == CoralConstants.coralPickUpSetPoint_Rad;
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
    public void setState(CoralConstants.ArmStates state) {
        this.state = state;
    }

    /**
     * Runs the logic for the current arm state.
     */
    private void runState() {
        switch (state) {
            case CORAL_PICKUP:
                runStateCoralPickup();
                break;
            case HAND_OFF:
                runStateHandOff();
                break;
            case STOWED:
                runStateStowed();
                break;
            case STOPPED:
                runStateStopped();
                break;
            case MOVING_DOWN:
                runMovingDown();
                break;
            case MOVING_UP:
                runMovingUp();
                break;
            default:
                break;
        }
    }

    private void runMovingUp() {
        setVoltage(12 + calculateFeedForward());
    }

    private void runMovingDown() {
        setVoltage(-12 + calculateFeedForward());
    }

    private void runStateStopped() {
        setVoltage(0 + calculateFeedForward());
    }

    private void runStateCoralPickup() {
        double setPoint = CoralConstants.coralPickUpSetPoint_Rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runStateHandOff() {
        double setPoint = CoralConstants.handOffSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint));
    }

    private void runStateStowed() {
        double setPoint = CoralConstants.stowSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    /**
     * Logs data to Shuffleboard.
     */
    private void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionUnitsLog.set(data.positionUnits);
        velocityUnitsLog.set(data.velocityUnits);
        inputVoltsLog.set(data.inputVolts);
        appliedVoltsLog.set(data.appliedVolts);
        currentAmpsLog.set(data.currentAmps);
        tempCelciusLog.set(data.tempCelcius);

        armLigament.setAngle(Math.toDegrees(data.positionUnits));

        stateLog.set(state.name());
    }

    private double calculateFeedForward() {
        double feedForward = CoralConstants.kG * Math.cos(data.positionUnits);
        return feedForward;
    }

    /**
     * Periodic method for updating arm behavior.
     */
    @Override
    public void periodic() {

        armIO.updateData(data);

        logData();

        runState();
    }

}
