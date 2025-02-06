package frc.robot.subsystems.elevator;

import java.util.Map;
import java.util.function.Consumer;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorData;
import frc.robot.subsystems.elevator.real.ElevatorSparkMax;
import frc.robot.subsystems.elevator.sim.ElevatorSimulation;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.MotorData;
import frc.robot.utils.UtilityFunctions;

import static edu.wpi.first.units.Units.*;

/**
 * Elevator subsystem
 * 
 * @author Dhyan Soni
 * @author Andrew Ge
 */

public class Elevator extends SubsystemBase {
    private ElevatorIO elevatorio;
    private ElevatorData data = new ElevatorData();
    private ElevatorStates state = ElevatorStates.STOP;

    static Consumer<Voltage> setVolts;
    static Consumer<SysIdRoutineLog> log;

    private ProfiledPIDController pidController = new ProfiledPIDController(
            ElevatorConstants.ElevatorControl.kPSim,
            0,
            ElevatorConstants.ElevatorControl.kDSim,
            new TrapezoidProfile.Constraints(ElevatorConstants.ElevatorControl.maxV,
                    ElevatorConstants.ElevatorControl.maxA));

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(
            ElevatorConstants.ElevatorControl.kSSim,
            ElevatorConstants.ElevatorControl.kGSim,
            ElevatorConstants.ElevatorControl.kVSim,
            ElevatorConstants.ElevatorControl.kASim);

    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    private LoggedTunableNumber positionMetersLog = new LoggedTunableNumber("Elevator//position", 0.0);
    private LoggedTunableNumber velocityMetersPerSecLog = new LoggedTunableNumber("Elevator/velocity", 0.0);
    private LoggedTunableNumber accelerationMetersPerSecSquaredLog = new LoggedTunableNumber("Elevator/acceleration", 0.0);

    private LoggedTunableNumber inputVoltsLog = new LoggedTunableNumber("Elevator/input volts", 0.0);
    private LoggedTunableNumber leftAppliedVoltsLog = new LoggedTunableNumber("Elevator/left applied volts", 0.0);
    private LoggedTunableNumber rightAppliedVoltsLog = new LoggedTunableNumber("Elevator/right applied volts", 0.0);
    private LoggedTunableNumber leftCurrentAmpsLog = new LoggedTunableNumber("Elevator/left current amps", 0.0);
    private LoggedTunableNumber rightCurrentAmpsLog = new LoggedTunableNumber("Elevator/right current amps", 0.0);
    private LoggedTunableNumber leftTempCelciusLog = new LoggedTunableNumber("Elevator/left temp celcius", 0.0);
    private LoggedTunableNumber rightTempCelciusLog = new LoggedTunableNumber("Elevator/right temp celcius", 0.0);

    // For tuning on real
    // private ShuffleData<Double> kPData = new ShuffleData<Double>("Elevator",
    // "kPData", ElevatorConstants.ElevatorControl.kPSim);
    // private ShuffleData<Double> kDData = new ShuffleData<Double>("Elevator",
    // "kDData", ElevatorConstants.ElevatorControl.kDSim);
    // private ShuffleData<Double> kGData = new ShuffleData<Double>("Elevator",
    // "kGData", ElevatorConstants.ElevatorControl.kGSim);
    // private ShuffleData<Double> kVData = new ShuffleData<Double>("Elevator",
    // "kVData", ElevatorConstants.ElevatorControl.kVSim);
    // private ShuffleData<Double> kAData = new ShuffleData<Double>("Elevator",
    // "kAData", ElevatorConstants.ElevatorControl.kASim);

    private Mechanism2d mech = new Mechanism2d(3, 3);
    private MechanismRoot2d root = mech.getRoot("elevator", 2, 0);
    private MechanismLigament2d elevatorMech = root
            .append(new MechanismLigament2d("elevator", ElevatorConstants.ElevatorSpecs.baseHeight, 90));

    // SysID
    Map<String, MotorData> motorData = Map.of(
            "elevator_motor", new MotorData(
                    (data.leftAppliedVolts + data.rightAppliedVolts) / 2.0,
                    data.positionMeters,
                    data.velocityMetersPerSecond,
                    data.accelerationUnits));

    private SysIdRoutine.Config config = new SysIdRoutine.Config(
        Volts.per(Seconds).of(1), // Voltage ramp rate
        Volts.of(7), // Max voltage
        Seconds.of(4) // Test duration
    );

    private SysIdTuner sysIdTuner;

    public Elevator() {
        if (Robot.isSimulation()) {
            elevatorio = new ElevatorSimulation();
        } else {
            elevatorio = new ElevatorSparkMax();
        }
        sysIdTuner = new SysIdTuner("elevator", config, this, elevatorio::setVoltage, motorData);
    }

    public SysIdTuner getSysIdTuner(){
        return sysIdTuner;
    }

    public ElevatorStates getState() {
        return state;
    }

    public double getPositionMeters() {
        return data.positionMeters;
    }

    public double getVelocityRadPerSec() {
        return data.velocityMetersPerSecond;
    }

    // returns true when the state is reached
    public boolean getIsStableState() {
        switch (state) {
            case L1:
                return UtilityFunctions.withinMargin(0.001, ElevatorConstants.StateHeights.l1Height,
                        data.positionMeters);
            case L2:
                return UtilityFunctions.withinMargin(0.001, data.positionMeters,
                        ElevatorConstants.StateHeights.l2Height);
            case L3:
                return UtilityFunctions.withinMargin(0.001, data.positionMeters,
                        ElevatorConstants.StateHeights.l3Height);
            case L4:
                return UtilityFunctions.withinMargin(0.001, data.positionMeters,
                        ElevatorConstants.StateHeights.l4Height);
            default:
                return false;
        }
    }

    public void setVoltage(double volts) {
        elevatorio.setVoltage(volts);
    }

    public void setState(ElevatorStates state) {
        this.state = state;
        switch (state) {
            case STOP:
                runStateStop();
                break;
            case L1:
                setGoal(ElevatorConstants.StateHeights.l1Height);
                break;
            case L2:
                setGoal(ElevatorConstants.StateHeights.l2Height);
                break;
            case L3:
                setGoal(ElevatorConstants.StateHeights.l3Height);
                break;
            case L4:
                setGoal(ElevatorConstants.StateHeights.l4Height);
                break;
            case MAX:
                setGoal(6);
                break;
            case STOW:
            default:
                setGoal(0);
                break;
        }
    }

    public void setGoal(double height) {
        pidController.setGoal(height);
    }

    private void runState() {
        switch (state) {
            case STOP:
                runStateStop();
                break;
            default:
                moveToGoal();
                break;
        }
    }

    private void moveToGoal() {
        State firstState = pidController.getSetpoint();
        double pidVoltage = pidController.calculate(getPositionMeters());

        State nextState = pidController.getSetpoint();
        double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

        elevatorio.setVoltage(ffVoltage + pidVoltage);
    }

    private void runStateStop() {
        stop();
    }

    public void stop() {
        elevatorio.setVoltage(0);
    }

    private void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionMetersLog.set(data.positionMeters);
        velocityMetersPerSecLog.set(data.velocityMetersPerSecond);
        accelerationMetersPerSecLog.set(data.accelerationUnits);
        inputVoltsLog.set(data.inputVolts);
        leftAppliedVoltsLog.set(data.leftAppliedVolts);
        rightAppliedVoltsLog.set(data.rightAppliedVolts);
        leftCurrentAmpsLog.set(data.leftCurrentAmps);
        rightCurrentAmpsLog.set(data.rightCurrentAmps);
        leftTempCelciusLog.set(data.leftTempCelcius);
        rightTempCelciusLog.set(data.rightTempCelcius);

        setpointVelocityLog.set(pidController.getSetpoint().velocity);
        setpointPositionLog.set(pidController.getSetpoint().position);

        elevatorMech.setLength(ElevatorConstants.ElevatorSpecs.baseHeight + data.positionMeters);
        SmartDashboard.putData("elevator mechanism", mech);
    }

    @Override
    public void periodic() {
        elevatorio.updateData(data);

        runState();
        logData();

        motorData.get("elevator_motor").position = data.positionMeters;
        motorData.get("elevator_motor").acceleration = data.accelerationUnits;
        motorData.get("elevator_motor").velocity = data.velocityMetersPerSecond;
        motorData.get("elevator_motor").appliedVolts = (data.leftAppliedVolts + data.rightAppliedVolts) / 2.0;

        // Map<String, MotorData> motorData = Map.of(
        //     "elevator_motor", new MotorData(
        //             (data.leftAppliedVolts + data.rightAppliedVolts) / 2.0,
        //             data.positionMeters,
        //             data.velocityMetersPerSecond,
        //             data.accelerationUnits));
                    
        // sysIdTuner = new SysIdTuner("elevator", config, this, elevatorio::setVoltage, motorData);
        // pidController.setPID(kPData.get(),0,kDData.get())
    }
}
