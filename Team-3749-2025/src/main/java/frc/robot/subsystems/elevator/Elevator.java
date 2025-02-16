package frc.robot.subsystems.elevator;

import java.util.Map;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
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
@SuppressWarnings("unused")
public class Elevator extends SubsystemBase {
    private ElevatorIO elevatorio;
    private ElevatorData data = new ElevatorData();
    private ElevatorStates state = ElevatorStates.STOP;


    private ProfiledPIDController profile = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(
            ElevatorConstants.ElevatorControl.maxVelocity.get(),
            ElevatorConstants.ElevatorControl.maxAcceleration.get()));

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.ElevatorControl.kS.get(),
            ElevatorConstants.ElevatorControl.kG.get(), ElevatorConstants.ElevatorControl.kV.get(),
            ElevatorConstants.ElevatorControl.kA.get());

    private LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    private LoggedMechanismRoot2d root = mech.getRoot("elevator", 1, 0);
    private LoggedMechanismLigament2d elevatorMech = root
            .append(new LoggedMechanismLigament2d("elevator", ElevatorConstants.ElevatorSpecs.baseHeight, 90));

    private double elevatorInnerStagePos;
    private double elevatorMiddleStagePos;

    StructPublisher<Pose3d> elevatorInnerStage = NetworkTableInstance.getDefault()
            .getStructTopic("Elevator Inner Stage", Pose3d.struct).publish();
    StructPublisher<Pose3d> elevatorMiddleStage = NetworkTableInstance.getDefault()
            .getStructTopic("Elevator Middle Stage", Pose3d.struct).publish();

    public Elevator() {
        if (Robot.isSimulation()) {
            elevatorio = new ElevatorSimulation();
        } else {
            elevatorio = new ElevatorSparkMax();
        }
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

    /** returns true when the state is reached */
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
                stop();
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
                setGoal(ElevatorConstants.ElevatorSpecs.maxHeightMeters);
                break;
            case STOW:
                setGoal(ElevatorConstants.ElevatorSpecs.baseHeight);
                break;
            default:
                setGoal(0);
                break;
        }
    }

    public void setGoal(double height) {
        profile.setGoal(height);
    }

    private void runState() {
        switch (state) {
            case STOP:
                stop();
                break;
            default:
                moveToGoal();
                break;
        }
    }

    private void moveToGoal() {
        State firstState = profile.getSetpoint();
        profile.calculate(getPositionMeters());

        State nextState = profile.getSetpoint();
        double ffVoltage = feedforward.calculate(firstState.velocity, nextState.velocity);

        elevatorio.setPosition(firstState.position, ffVoltage);
    }

    public void stop() {
        elevatorio.setVoltage(0);
    }

    private void logData() {
 
        Logger.recordOutput("subsystems/elevator/Current Command",
                this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());

        Logger.recordOutput("subsystems/elevator/postion", data.positionMeters);
        Logger.recordOutput("subsystems/elevator/velocity", data.velocityMetersPerSecond);

        Logger.recordOutput("subsystems/elevator/acceleration", data.accelerationMetersPerSecondSquared);

        Logger.recordOutput("subsystems/elevator/input volts",
                ((data.leftAppliedVolts + data.rightAppliedVolts) / 2.0));
        Logger.recordOutput("subsystems/elevator/leftAppliedVolts", data.leftAppliedVolts);
        Logger.recordOutput("subsystems/elevator/rightAppliedVolts", data.rightAppliedVolts);
        Logger.recordOutput("subsystems/elevator/leftCurrentAmps", data.leftCurrentAmps);
        Logger.recordOutput("subsystems/elevator/rightCurrentAmps", data.rightCurrentAmps);
        Logger.recordOutput("subsystems/elevator/leftTempCelcius", data.leftTempCelcius);
        Logger.recordOutput("subsystems/elevator/rightTempCelcius", data.rightTempCelcius);
        Logger.recordOutput("subsystems/elevator/state", state.name());

        elevatorMech.setLength(ElevatorConstants.ElevatorSpecs.baseHeight + data.positionMeters);

        elevatorInnerStagePos = data.positionMeters / 2;
        elevatorMiddleStagePos = data.positionMeters - Units.inchesToMeters(1);
        elevatorInnerStage.set(new Pose3d(getTransform3d(elevatorInnerStagePos).getTranslation(),
                getTransform3d(elevatorInnerStagePos).getRotation()));
        elevatorMiddleStage.set(new Pose3d(getTransform3d(elevatorMiddleStagePos).getTranslation(),
                getTransform3d(elevatorMiddleStagePos).getRotation()));

        Logger.recordOutput("subsystems/elevator/elevator mechanism", mech);
    }

    private Transform3d getTransform3d(double pos) {
        Transform3d transform = new Transform3d(0, 0, pos, new Rotation3d(Angle.ofBaseUnits(0, Radians),
                Angle.ofBaseUnits(0, Radians), Angle.ofBaseUnits(0, Radians)));
        return transform;
    }

    @Override
    public void periodic() {
        elevatorio.updateData(data);
        runState();
        logData();
    }
}
