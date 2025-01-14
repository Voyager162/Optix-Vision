package frc.robot.subsystems.arm.armJavaFiles;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmConstants.algeaArmConstants;
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

public class AlgeaArm extends SubsystemBase {

    private ArmIO armIO;
    private ArmData data = new ArmData();
    private algeaArmConstants.ArmStates state = algeaArmConstants.ArmStates.STOPPED;

    private double setPoint;

    private PIDController controller = new PIDController
    (
        algeaArmConstants.kP, 
        algeaArmConstants.kI, 
        algeaArmConstants.kD
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
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Algea Arm", 17, 0));

    public ShuffleData<Double> kGLog = new ShuffleData<Double>(this.getName(), "kG", 0.0);
    public ShuffleData<Double> kPLog = new ShuffleData<Double>(this.getName(), "kP", 0.0);



    public AlgeaArm() {
        if (Robot.isSimulation()) {

            armIO = new ArmSim
            (1, 
            algeaArmConstants.armGearing, 
            algeaArmConstants.momentOfInertia, 
            algeaArmConstants.armLength_meters, 
            algeaArmConstants.armMinAngle_degrees, 
            algeaArmConstants.armMaxAngle_degrees, 
            algeaArmConstants.simulateGravity, 
            algeaArmConstants.armStartingAngle_degrees
            );

        } else {
            armIO = new ArmSparkMax(algeaArmConstants.motorId);
        }
        SmartDashboard.putData("Algea Arm Mechanism", mechanism2d);
    }

    public double getPositionRad() {
        return data.positionUnits;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public algeaArmConstants.ArmStates getState() {
        return state;
    }

    public boolean getIsStableState() {

        switch (state) {
            case STOWED:
                return data.positionUnits == algeaArmConstants.stowSetPoint_rad;
            case PROCESSOR:
                return data.positionUnits == algeaArmConstants.processorSetPoint_rad;
            case ALGEA_PICKUP:
                return data.positionUnits == algeaArmConstants.algeaPickUpSetPoint_rad;
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

    public void setState(algeaArmConstants.ArmStates state) {
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
            case ALGEA_PICKUP:
                runStateAlgeaPickup();
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
        setPoint = algeaArmConstants.stowSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runStateProccessor() {
        setPoint = algeaArmConstants.processorSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runStateAlgeaPickup() {
        setPoint = algeaArmConstants.algeaPickUpSetPoint_rad;
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

        algeaArmConstants.kG = kGLog.get();
        algeaArmConstants.kP = kPLog.get();

        controller = new PIDController
        (
            algeaArmConstants.kP, 
            algeaArmConstants.kI, 
            algeaArmConstants.kD
        );

        armIO.updateData(data);

        logData();

        runState();
    }

    private double calculateFeedForward() {
        double feedForward = algeaArmConstants.kG * Math.cos(data.positionUnits);
        return feedForward;
    }

}
