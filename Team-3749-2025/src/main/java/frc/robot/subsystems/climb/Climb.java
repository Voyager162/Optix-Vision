package frc.robot.subsystems.climb;

import java.util.Map;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import frc.robot.subsystems.climb.ClimbIO.ClimbData;
import frc.robot.subsystems.climb.real.ClimbSparkMax;
import frc.robot.subsystems.climb.sim.ClimbSim;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorData;
import frc.robot.subsystems.elevator.real.ElevatorSparkMax;
import frc.robot.subsystems.elevator.sim.ElevatorSimulation;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.SysIdTuner;
import frc.robot.utils.MotorData;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.SysIdTuner.Type;

import static edu.wpi.first.units.Units.*;

/**
 * Elevator subsystem
 * 
 * @author Dhyan Soni
 * @author Andrew Ge
 */
@SuppressWarnings("unused")
public class Climb extends SubsystemBase {
    private ClimbIO climbIO;
    private ClimbData climbData = new ClimbData();
    private ProfiledPIDController climbPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(
            ClimbConstants.ClimbControl.climbMaxSpeedRadPerSec,
            ClimbConstants.ClimbControl.climbMaxAccelRadPerSec));

    private ElevatorFeedforward climbFF = new SimpleMotorFeedforward(ElevatorConstants.ElevatorControl.kS.get(),
            ElevatorConstants.ElevatorControl.kG.get(), ElevatorConstants.ElevatorControl.kV.get(),
            ElevatorConstants.ElevatorControl.kA.get());

    public Climb() {
        if (Robot.isSimulation()) {
            climbIO = new ClimbSim();
        } else {
            climbIO = new ClimbSparkMax();
        }
    }

    public void setVoltage(double volts) {
        climbIO.setVoltage(volts);
    }

    public double getVelocity()
    {
        return climbData.velocityRadiansPerSecond;
    }

    public void setGoal(double velocityRadPerSec)
    {
        climbPID.setGoal(velocityRadPerSec);
    }

    public void setIsClimbing(boolean isClimbing)
    {
        climbIO.setIsClimbing(isClimbing);
    }

    public void stop() {
        setVoltage(0);
    }

    private void logData() {
        // Logger.recordOutput("Climb/positionRadians",climbData.positionRadians); 
        Logger.recordOutput("Climb/velocityRadPerSec",climbData.velocityRadiansPerSecond);
        Logger.recordOutput("Climb/isClimbing", climbData.isClimbing); 
    }

    private void moveToGoal()
    {
        State firstState = climbPID.getGoal();
        double PID = climbPID.calculate(getVelocity());

        State nextState = climbPID.getSetpoint();
        double FF = climbFF.calculateWithVelocities(firstState.velocity, nextState.velocity);
        setVoltage(FF + PID);
    }

    @Override
    public void periodic() {
        climbIO.updateData(climbData);
        if(climbData.isClimbing)
        {
           moveToGoal();
        }
        logData();
    }
}
