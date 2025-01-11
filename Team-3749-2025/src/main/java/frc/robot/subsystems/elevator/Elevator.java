package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorData;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;

public class Elevator extends SubsystemBase {
    // armIO.setVoltage(pidController.calculate(getPosition(), setpoint) + ArmConstants.armControl.armkG);
    
    private ElevatorIO elevatorio;
    private ElevatorData data = new ElevatorData();
    private ElevatorStates state = ElevatorStates.STOP;
    private ElevatorConstants constants = new ElevatorConstants();
    
    private ProfiledPIDController pidController = new ProfiledPIDController(
        ElevatorConstants.ElevatorControl.kPSim, 
        0, 
        ElevatorConstants.ElevatorControl.kDSim,
        new TrapezoidProfile.Constraints(ElevatorConstants.ElevatorControl.kVSim, ElevatorConstants.ElevatorControl.kASim));
    // maybe later
    // private ElevatorFeedforward feedForward = new ElevatorFeedforward(ElevatorConstants.ElevatorControl.kASim, ElevatorConstants.ElevatorControl.kGSim, ElevatorConstants.ElevatorControl.kSSim, ElevatorConstants.ElevatorControl.kASim);

    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    private ShuffleData<Double> positionMetersLog = new ShuffleData<Double>("Elevator", "position meters", 0.0);
    private ShuffleData<Double> velocityUnitsLog = new ShuffleData<Double>("Elevator", "velocity units", 0.0);
    private ShuffleData<Double> accelerationUnitsLog = new ShuffleData<Double>("Elevator", "acceleration units", 0.0);
    private ShuffleData<Double> inputVoltsLog = new ShuffleData<Double>("Elevator", "input volts", 0.0);
    private ShuffleData<Double> appliedVoltsLog = new ShuffleData<Double>("Elevator", "applied volts", 0.0);
    private ShuffleData<Double> currentAmpsLog = new ShuffleData<Double>("Elevator", "current amps", 0.0);
    private ShuffleData<Double> tempCelciusLog = new ShuffleData<Double>("Elevator", "temp celcius", 0.0);

    // Temporary?
    // private ShuffleData<Double> kPData = new ShuffleData<Double>("Elevator", "kPData", ElevatorConstants.ElevatorControl.kPSim);
    // private ShuffleData<Double> kDData = new ShuffleData<Double>("Elevator", "kDData", ElevatorConstants.ElevatorControl.kDSim); 
    // private ShuffleData<Double> kGData = new ShuffleData<Double>("Elevator", "kGData", ElevatorConstants.ElevatorControl.kGSim); 

    private Mechanism2d mech = new Mechanism2d(3, 3);
    private MechanismRoot2d root = mech.getRoot("elevator", 2, 0);
    private MechanismLigament2d elevatorMech = root.append(new MechanismLigament2d("elevator", Units.feetToMeters(3.25), 90));

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
                runStateStop();
                break;
            case L1:
                setHeight(constants.l1Height);
                break;
            case L2:
                setHeight(constants.l2Height);
                break;
            case L3:
                setHeight(constants.l3Height);
                break;
            case L4:
                setHeight(constants.l4Height);
                break;
            case MAX:
                break;
            case STOW:
                break;
        }
    }

    private void runStateStop() {
        elevatorio.setVoltage(0);
    }

    // returns true when the state is reached
    public boolean getIsStableState() {
        switch (state) {
            case STOP:
                return data.velocityUnits == 0;
            case L1:
                return UtilityFunctions.withinMargin(0.1, data.positionMeters, Units.inchesToMeters(constants.l1Height)); 
            case L2:
                return UtilityFunctions.withinMargin(0.1, data.positionMeters, Units.inchesToMeters(constants.l2Height));
            case L3:
                return UtilityFunctions.withinMargin(0.1, data.positionMeters, Units.inchesToMeters(constants.l3Height));
            case L4:
                return UtilityFunctions.withinMargin(0.1, data.positionMeters, Units.inchesToMeters(constants.l4Height));
            default:
                return false;
        }
    }
    

    public ElevatorStates getState() {
        return state;
    }

    public double getPositionMeters() {
        return data.positionMeters;
    }

    public double getVelocityRadPerSec() {
        return data.velocityUnits;
    }

    public void stop() {
        elevatorio.setVoltage(0);
    }

    private void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionMetersLog.set(data.positionMeters);
        velocityUnitsLog.set(data.velocityUnits);
        inputVoltsLog.set(data.inputVolts);
        appliedVoltsLog.set(data.appliedVolts);
        currentAmpsLog.set(data.currentAmps);
        tempCelciusLog.set(data.tempCelcius);
    }

    public void setVoltage(double volts){
        elevatorio.setVoltage(volts);
    }

    private void setHeight(double setpoint){
        elevatorio.setVoltage(pidController.calculate(getPositionMeters(), setpoint) + ElevatorConstants.ElevatorControl.kGSim);
    }

    public void setState(ElevatorStates state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        elevatorio.updateData(data);

        System.out.println(state);
        runState();
        logData();
        // pidController.setP(kPData.get());
        // pidController.setD(kDData.get());
        elevatorMech.setLength(Units.feetToMeters(3.25) + data.positionMeters);
        SmartDashboard.putData("elevator mechanism", mech);
    }
}
