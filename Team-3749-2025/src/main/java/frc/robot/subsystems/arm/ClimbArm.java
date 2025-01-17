package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.climbArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.subsystems.arm.real.ClimbSparkMax;
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

public class ClimbArm extends SubsystemBase {

    private ArmIO armIO;
    private ArmData data = new ArmData();
    private climbArmConstants.ArmStates state = climbArmConstants.ArmStates.STOPPED;

    private double setPoint;

    private PIDController controller = new PIDController
    (
        climbArmConstants.kP, 
        climbArmConstants.kI, 
        climbArmConstants.kD
    );

    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    public ShuffleData<Double> positionUnitsLog = new ShuffleData<Double>(this.getName(), "position units", 0.0);
    public ShuffleData<Double> velocityUnitsLog = new ShuffleData<Double>(this.getName(), "velocity units", 0.0);
    public ShuffleData<Double> accelerationUnitsLog = new ShuffleData<Double>(this.getName(), "acceleration units", 0.0);
    public ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>(this.getName(), "input volts", 0.0);
    public ShuffleData<Double> appliedVoltsLog = new ShuffleData<Double>(this.getName(), "applied volts", 0.0);
    public ShuffleData<Double> currentAmpsLog = new ShuffleData<Double>(this.getName(), "current amps", 0.0);
    public ShuffleData<Double> tempCelciusLog = new ShuffleData<Double>(this.getName(), "temp celcius", 0.0);

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Climb Arm", 24, 0));

    public ShuffleData<Double> kGLog = new ShuffleData<Double>(this.getName(), "kG", 0.0);
    public ShuffleData<Double> kPLog = new ShuffleData<Double>(this.getName(), "kP", 0.0);



    public ClimbArm() {
        if (Robot.isSimulation()) {
            
            armIO = new ArmSim
            (
                climbArmConstants.numMotors, 
                climbArmConstants.armGearing, 
                climbArmConstants.momentOfInertia, 
                climbArmConstants.armLength_meters, 
                climbArmConstants.armMinAngle_degrees, 
                climbArmConstants.armMaxAngle_degrees, 
                climbArmConstants.simulateGravity, 
                climbArmConstants.armStartingAngle_degrees
            );

        } else {
            armIO = new ClimbSparkMax(climbArmConstants.firstMotorId, climbArmConstants.secondMotorId);
        }
        SmartDashboard.putData("Climb Arm Mechanism", mechanism2d);
    }

    public double getPositionRad() {
        return data.positionUnits;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public climbArmConstants.ArmStates getState() {
        return state;
    }

    public boolean getIsStableState() {

        switch (state) {
            case STOWED:
                return data.positionUnits == climbArmConstants.stowSetPoint_rad;
            case PREPARE_FOR_CLIMB:
                return data.positionUnits == climbArmConstants.PrepareForClimbSetPoint_Rad;
            case CLIMB:
                return data.positionUnits == climbArmConstants.climbSetPoint_rad;
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

    public void setState(climbArmConstants.ArmStates state) {
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
            case PREPARE_FOR_CLIMB:
                runStatePrepareForClimb();
                break;
            case CLIMB:
                runStateClimb();
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
        setVoltage(1 + calculateFeedForward());
    }

    private void runMovingDown() {
        setVoltage(-1 + calculateFeedForward());
    }

    private void runStateStopped() {
        setVoltage(0 + calculateFeedForward());
    }

    private void runStatePrepareForClimb() {
        setPoint = climbArmConstants.PrepareForClimbSetPoint_Rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runStateClimb() {
        setPoint = climbArmConstants.climbSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runStateStowed() {
        setPoint = climbArmConstants.stowSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
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

        climbArmConstants.kG = kGLog.get();
        climbArmConstants.kP = kPLog.get();

        controller = new PIDController
        (
            climbArmConstants.kP, 
            climbArmConstants.kI, 
            climbArmConstants.kD

        );

        armIO.updateData(data);

        logData();

        runState();
    }

    private double calculateFeedForward() {
        double feedForward = climbArmConstants.kG * Math.cos(data.positionUnits);
        return feedForward;
    }

}
