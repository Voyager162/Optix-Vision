package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO.ArmData;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.utils.ShuffleData;

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


    public Arm() {
        if (Robot.isSimulation()) {
            armIO = new ArmSim();
        } 
        else {
            // Initialize armIO for real robot
        }
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

    private void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionUnitsLog.set(data.positionUnits);
        velocityUnitsLog.set(data.velocityUnits);
        inputVoltsLog.set(data.inputVolts);
        appliedVoltsLog.set(data.appliedVolts);
        currentAmpsLog.set(data.currentAmps);
        tempCelciusLog.set(data.tempCelcius);

    }




    @Override
    public void periodic() {
        armIO.updateData(data);

        logData();
    }

}
