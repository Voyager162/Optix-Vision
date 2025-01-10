package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorData;
import frc.robot.utils.ShuffleData;

public class Elevator extends SubsystemBase {
    ElevatorIO elevatorio;
    ElevatorData data = new ElevatorData();
    private ElevatorStates state = ElevatorStates.STOP;

    PIDController pidController = new PIDController(ElevatorConstants.ElevatorControl.kPSim, 0, ElevatorConstants.ElevatorControl.kDSim); 

    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    public ShuffleData<Double> positionUnitsLog = new ShuffleData<Double>("Elevator", "position units", 0.0);
    public ShuffleData<Double> velocityUnitsLog = new ShuffleData<Double>("Elevator", "velocity units", 0.0);
    public ShuffleData<Double> accelerationUnitsLog = new ShuffleData<Double>("Elevator", "acceleration units", 0.0);
    public ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>("Elevator", "input volts", 0.0);
    public ShuffleData<Double> appliedVoltsLog = new ShuffleData<Double>("Elevator", "applied volts", 0.0);
    public ShuffleData<Double> currentAmpsLog = new ShuffleData<Double>("Elevator", "current amps", 0.0);
    public ShuffleData<Double> tempCelciusLog = new ShuffleData<Double>("Elevator", "temp celcius", 0.0);

    // Temporary?
    private ShuffleData<Double> kPData = new ShuffleData<Double>("Elevator", "kPData", ElevatorConstants.ElevatorControl.kPSim);
    private ShuffleData<Double> kDData = new ShuffleData<Double>("Elevator", "kDData", ElevatorConstants.ElevatorControl.kPSim); 

    public static Mechanism2d mech = new Mechanism2d(3, 3);
    public static MechanismRoot2d root = mech.getRoot("elevator", 2, 0);
    public static MechanismLigament2d elevatorMech = root.append(new MechanismLigament2d("elevator", Units.feetToMeters(3.25), 90));

    public Elevator(){
        if (Robot.isSimulation()) {
            elevatorio = new ElevatorSimulation();
        } else {
            elevatorio = new ElevatorSparkMax();
        }
    }

    private void runState() {
        switch (state) {
            case STOP:
                break;
            case GO:
                break;
            case L1:
                break;
            case L2:
                break;
            case L3:
                break;
            case L4:
                break;
            case MAX:
                break;
        }
    }

    public ElevatorStates getState() {
        return state;
    }

    public double getPositionRad() {
        return data.positionUnits;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public void stop() {
        elevatorio.setVoltage(0);
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
        elevatorio.updateData(data);
        runState();
        logData();
        pidController.setP(kPData.get());
        pidController.setD(kDData.get());
    }
}
