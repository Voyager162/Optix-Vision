package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.subsystems.arm.real.ArmSparkMax;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.Robot;
import frc.robot.utils.UtilityFunctions;

public abstract class ArmSubsystem extends SubsystemBase {

    protected ArmIO armIO;
    protected ArmData data = new ArmData();
    protected PIDController controller;
    protected ShuffleData<String> currentCommandLog = new ShuffleData<>(this.getName(), "current command", "None");
    protected ShuffleData<Double> positionUnitsLog = new ShuffleData<>(this.getName(), "position units", 0.0);
    protected ShuffleData<Double> velocityUnitsLog = new ShuffleData<>(this.getName(), "velocity units", 0.0);
    protected ShuffleData<Double> appliedVoltsLog = new ShuffleData<>(this.getName(), "applied volts", 0.0);
    protected ShuffleData<Double> currentAmpsLog = new ShuffleData<>(this.getName(), "current amps", 0.0);
    protected ShuffleData<Double> tempCelciusLog = new ShuffleData<>(this.getName(), "temp celcius", 0.0);
    protected ShuffleData<String> stateLog = new ShuffleData<>(this.getName(), "state", "STOPPED");

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Arm", 24, 0));

    protected double setPoint;
    protected ArmConstants.ArmStates state;

    public ArmSubsystem() {
        if (Robot.isSimulation()) {
            armIO = new ArmSim(
                ArmConstants.numMotors,
                ArmConstants.armGearing,
                ArmConstants.momentOfInertia,
                ArmConstants.armLength_meters,
                ArmConstants.armMinAngle_degrees,
                ArmConstants.armMaxAngle_degrees,
                ArmConstants.simulateGravity,
                ArmConstants.armStartingAngle_degrees
            );
        } else {
            armIO = new ArmSparkMax(ArmConstants.motorId);
        }
        SmartDashboard.putData("Arm Mechanism", mechanism2d);
        
        controller = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        state = ArmConstants.ArmStates.STOPPED;
    }

    // Abstract method for specific state handling
    protected abstract void runStateLogic();

    // Method to set the voltage for the arm
    protected void setVoltage(double volts) {
        armIO.setVoltage(volts);
    }

    // Calculate the feedforward term for gravity compensation
    protected double calculateFeedForward() {
        return ArmConstants.kG * Math.cos(data.positionUnits);
    }

    // Common state machine method
    protected void runState() {
        runStateLogic();
    }

    // Method to update arm data
    protected void updateData() {
        armIO.updateData(data);
    }

    // Method to log common arm data
    protected void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionUnitsLog.set(data.positionUnits);
        velocityUnitsLog.set(data.velocityUnits);
        appliedVoltsLog.set(data.appliedVolts);
        currentAmpsLog.set(data.currentAmps);
        tempCelciusLog.set(data.tempCelcius);
        
        armLigament.setAngle(Math.toDegrees(data.positionUnits));

        stateLog.set(state.name());
    }

    @Override
    public void periodic() {
        updateData();
        logData();
        runState();
    }

    public double getPositionRad() {
        return data.positionUnits;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public ArmConstants.ArmStates getState() {
        return state;
    }

    public boolean getIsStableState() {
        switch (state) {
            case STOWED:
                return data.positionUnits == ArmConstants.stowSetPoint_rad;
            case HAND_OFF:
                return data.positionUnits == ArmConstants.handOffSetPoint_rad;
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

    public void setState(ArmConstants.ArmStates newState) {
        this.state = newState;
    }
}
