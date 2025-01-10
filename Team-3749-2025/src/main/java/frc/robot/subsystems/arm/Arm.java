package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;
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

public class Arm extends SubsystemBase {


    private ArmIO armIO;
    private ArmData data = new ArmData();
    private ArmStates state = ArmStates.STOPPED;

    private double setPoint;

    private PIDController controller = new PIDController(ArmConstants.Kp, ArmConstants.Ki, ArmConstants.Kd);
    
    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    public ShuffleData<Double> positionUnitsLog = new ShuffleData<Double>("Arm", "position units", 0.0);
    public ShuffleData<Double> velocityUnitsLog = new ShuffleData<Double>("Arm", "velocity units", 0.0);
    public ShuffleData<Double> accelerationUnitsLog = new ShuffleData<Double>("Arm", "acceleration units", 0.0);
    public ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>("Arm", "input volts", 0.0);
    public ShuffleData<Double> appliedVoltsLog = new ShuffleData<Double>("Arm", "applied volts", 0.0);
    public ShuffleData<Double> currentAmpsLog = new ShuffleData<Double>("Arm", "current amps", 0.0);
    public ShuffleData<Double> tempCelciusLog = new ShuffleData<Double>("Arm", "temp celcius", 0.0);

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Arm", 30, 0));

    private double Voltage = 0;

    public Arm() {
        if (Robot.isSimulation()) {
            armIO = new ArmSim();
        } 
        else {
            // Initialize armIO for real robot
        }
        SmartDashboard.putData("Arm Mechanism", mechanism2d);
    }

    public double getPositionRad() {
        return data.positionUnits;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public ArmStates getState() {
        return state;
    }

    public boolean getIsStableState() {

        switch (state) {
            case MOVING:
                return true;
            case HALFWAY_EXTENDED:
                return data.positionUnits == (Math.PI * 3) / 4;
            case FULLY_EXTENDED:
                return data.positionUnits == Math.PI;
            case STOWED:
                return data.positionUnits == Math.PI / 2;
            case STOPPED:
                return data.velocityUnits == 0;
            default:
                return false;
        }
    }

    public void setState(ArmStates state) {
        this.state = state;
    }

    public double getAppliedVoltage() {
        return data.appliedVolts;
    }

    public void setVoltage(double volts) {
        this.Voltage = volts;
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

    private void runState() {
        switch (state) {
            case MOVING:
                runStateMoving();
                break;
            case FULLY_EXTENDED:
                runStateFullyExtended();
                break;
            case HALFWAY_EXTENDED:
                runStateHalfway();
                break;
            case STOWED:
                runStateStowed();
                break;
            case STOPPED:
                runStateStopped();
                break;
            default:
                break;
        }
    }

    private void runStateMoving() {
        // no action needed
    }

    private void runStateStopped() {
        setVoltage(0);
    }

    private void runStateFullyExtended() {
        setPoint = Math.PI;
        setVoltage(controller.calculate(data.positionUnits, setPoint));
    }

    private void runStateHalfway() {
        setPoint = (Math.PI * 3) / 4;
        setVoltage(controller.calculate(data.positionUnits, setPoint));
    }

    private void runStateStowed() {
        setPoint = Math.PI / 2;
        setVoltage(controller.calculate(data.positionUnits, setPoint));
    }

    @Override
    public void periodic() {
        armIO.updateData(data);
        armIO.setVoltage(Voltage + calculateFeedForward());

        logData();

        runState();
    }

    private double calculateFeedForward() {
        return ArmConstants.Kg * Math.cos(data.positionUnits);
    }

}
