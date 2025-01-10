package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;
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
    
    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    public ShuffleData<Double> positionUnitsLog = new ShuffleData<Double>("Arm", "position units", 0.0);
    public ShuffleData<Double> velocityUnitsLog = new ShuffleData<Double>("Arm", "velocity units", 0.0);
    public ShuffleData<Double> accelerationUnitsLog = new ShuffleData<Double>("Arm", "acceleration units", 0.0);
    public ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>("Arm", "input volts", 0.0);
    public ShuffleData<Double> appliedVoltsLog = new ShuffleData<Double>("Arm", "applied volts", 0.0);
    public ShuffleData<Double> currentAmpsLog = new ShuffleData<Double>("Arm", "current amps", 0.0);
    public ShuffleData<Double> tempCelciusLog = new ShuffleData<Double>("Arm", "temp celcius", 0.0);
    public ShuffleData<Double> kGLog = new ShuffleData<Double>("Arm", "kG", 4.0);

    private Mechanism2d mechanism2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 30, 30);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Arm", 30, 0));

    private double commandedVoltage = 0;

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

    public void setVoltage(double volts) {
        armIO.setVoltage(volts);
    }

    public double getAppliedVoltage() {
        return data.appliedVolts;
    }

    public void setCommandedVoltage(double volts) {
        this.commandedVoltage = volts;
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
    }

    @Override
    public void periodic() {
        ArmConstants.Kg = kGLog.get();
        armIO.updateData(data);
        double gravityCompensation = ArmConstants.Kg * Math.cos(data.positionUnits);
        armIO.setVoltage(commandedVoltage + gravityCompensation);
        logData();
    }

}
