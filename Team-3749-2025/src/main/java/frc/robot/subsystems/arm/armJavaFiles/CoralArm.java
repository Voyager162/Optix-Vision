package frc.robot.subsystems.arm.armJavaFiles;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmConstants.coralArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.subsystems.arm.real.ArmSparkMax;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem class for the arm
 * 
 * @author Weston Gardner
 */

public class CoralArm extends SubsystemBase {

    private ArmIO armIO;
    private ArmData data = new ArmData();
    private coralArmConstants.ArmStates state = coralArmConstants.ArmStates.STOPPED;

    private double setPoint;

    private ProfiledPIDController controller = new ProfiledPIDController(coralArmConstants.kPSim, coralArmConstants.kISim, coralArmConstants.kDSim, new TrapezoidProfile.Constraints(coralArmConstants.maxVelocity, coralArmConstants.maxAcceleration));

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
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Coral Arm", 24, 0));



    public CoralArm() {
        if (Robot.isSimulation()) {
            
            armIO = new ArmSim
            (
                coralArmConstants.numMotors, 
                coralArmConstants.armGearing, 
                coralArmConstants.momentOfInertia, 
                coralArmConstants.armLength_meters, 
                coralArmConstants.armMinAngle_degrees, 
                coralArmConstants.armMaxAngle_degrees, 
                coralArmConstants.simulateGravity, 
                coralArmConstants.armStartingAngle_degrees
            );

        } else {
            armIO = new ArmSparkMax(coralArmConstants.motorId);
        }
        SmartDashboard.putData("Coral Arm Mechanism", mechanism2d);
    }

    public double getPositionRad() {
        return data.positionUnits;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public coralArmConstants.ArmStates getState() {
        return state;
    }

    public boolean getIsStableState() {

        switch (state) {
            case STOWED:
                return data.positionUnits == coralArmConstants.stowSetPoint_rad;
            case HAND_OFF:
                return data.positionUnits == coralArmConstants.handOffSetPoint_rad;
            case CORAL_PICKUP:
                return data.positionUnits == coralArmConstants.coralPickUpSetPoint_Rad;
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

    public void setState(coralArmConstants.ArmStates state) {
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
        setVoltage(1 + calculateFeedForward());
    }

    private void runMovingDown() {
        setVoltage(-1 + calculateFeedForward());
    }

    private void runStateStopped() {
        setVoltage(0 + calculateFeedForward());
    }

    private void runStateCoralPickup() {
        setPoint = coralArmConstants.coralPickUpSetPoint_Rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runStateHandOff() {
        setPoint = coralArmConstants.handOffSetPoint_rad;
        setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
    }

    private void runStateStowed() {
        setPoint = coralArmConstants.stowSetPoint_rad;
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
        armIO.updateData(data);

        logData();

        runState();
    }

    private double calculateFeedForward() {
        return coralArmConstants.kGSim * Math.cos(data.positionUnits);
    }

}
