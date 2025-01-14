package frc.robot.subsystems.arm.armJavaFiles;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmConstants.algaeArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmData;
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

public class AlgaeArm extends SubsystemBase {

    private ArmIO armIO;
    private ArmData data = new ArmData();
    private algaeArmConstants.ArmStates state = algaeArmConstants.ArmStates.STOPPED;

    private double setPoint;

    private PIDController controller = new PIDController
    (
        algaeArmConstants.kP, 
        algaeArmConstants.kI, 
        algaeArmConstants.kD
    );

    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    public ShuffleData<Double> positionUnitsLog = new ShuffleData<Double>(this.getName(), "position units", 0.0);
    public ShuffleData<Double> velocityUnitsLog = new ShuffleData<Double>(this.getName(), "velocity units", 0.0);
    public ShuffleData<Double> accelerationUnitsLog = new ShuffleData<Double>(this.getName(), "acceleration units",0.0);
    public ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>(this.getName(), "input volts", 0.0);
    public ShuffleData<Double> appliedVoltsLog = new ShuffleData<Double>(this.getName(), "applied volts", 0.0);
    public ShuffleData<Double> currentAmpsLog = new ShuffleData<Double>(this.getName(), "current amps", 0.0);
    public ShuffleData<Double> tempCelciusLog = new ShuffleData<Double>(this.getName(), "temp celcius", 0.0);

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Algae Arm", 17, 0));

    public ShuffleData<Double> kGLog = new ShuffleData<Double>(this.getName(), "kG", 0.0);
    public ShuffleData<Double> kPLog = new ShuffleData<Double>(this.getName(), "kP", 0.0);



    public AlgaeArm() {
        if (Robot.isSimulation()) {

            armIO = new ArmSim
            (1, 
            algaeArmConstants.armGearing, 
            algaeArmConstants.momentOfInertia, 
            algaeArmConstants.armLength_meters, 
            algaeArmConstants.armMinAngle_degrees, 
            algaeArmConstants.armMaxAngle_degrees, 
            algaeArmConstants.simulateGravity, 
            algaeArmConstants.armStartingAngle_degrees
            );

        } else {
            armIO = new ArmSparkMax(algaeArmConstants.motorId);
        }
        SmartDashboard.putData("Algae Arm Mechanism", mechanism2d);
    }

    public double getPositionRad() {
        return data.positionUnits;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public algaeArmConstants.ArmStates getState() {
        return state;
    }

    public boolean getIsStableState() {

        switch (state) {
            case STOWED:
                return data.positionUnits == algaeArmConstants.stowSetPoint_rad;
            case PROCESSOR:
                return data.positionUnits == algaeArmConstants.processorSetPoint_rad;
            case ALGAE_PICKUP:
                return data.positionUnits == algaeArmConstants.algaePickUpSetPoint_rad;
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

    public void setState(algaeArmConstants.ArmStates state) {
        this.state = state;
    }

    public double getAppliedVoltage() {
        return data.appliedVolts;
    }

    public void setVoltage(double volts) {
        // this.voltage = volts;
        armIO.setVoltage(volts);
    }

    private void runState() {
        switch (state) {
            case STOWED:
                runStateStowed();
                break;
            case PROCESSOR:
                runStateProccessor();
                break;
            case ALGAE_PICKUP:
                runStateAlgaePickup();
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

    private void runStateStowed() {
        setPoint = algaeArmConstants.stowSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runStateProccessor() {
        setPoint = algaeArmConstants.processorSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runStateAlgaePickup() {
        setPoint = algaeArmConstants.algaePickUpSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runMovingUp() {
        setVoltage(1 + calculateFeedForward());
    }

    private void runMovingDown() {
        setVoltage(-1 + calculateFeedForward());
    }

    private void runStateStopped() {
        setVoltage(0 + calculateFeedForward());
    }

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

    @Override
    public void periodic() {

        algaeArmConstants.kG = kGLog.get();
        algaeArmConstants.kP = kPLog.get();

        controller = new PIDController
        (
            algaeArmConstants.kP, 
            algaeArmConstants.kI, 
            algaeArmConstants.kD
        );

        armIO.updateData(data);

        logData();

        runState();
    }

    private double calculateFeedForward() {
        double feedForward = algaeArmConstants.kG * Math.cos(data.positionUnits);
        return feedForward;
    }

}
