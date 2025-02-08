package frc.robot.subsystems.roller;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.real.RollerSparkMax;
import frc.robot.subsystems.roller.sim.RollerSim;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.MotorData;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.SysIdTuner;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

import static edu.wpi.first.units.Units.*;

public abstract class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerData rollerData = new RollerData();
    private RollerStates rollerState;
    private SimpleMotorFeedforward rollerFF;
    private double lastKnownPosition = 0.0;

    private LoggedTunableNumber rollerVelocityLog;
    private LoggedTunableNumber rollerVoltageLog;
    private LoggedTunableNumber rollerCurrentLog;
    private ShuffleData<String> stateLog;
    private LoggedTunableNumber rollerPositionLog;
    private LoggedTunableNumber rollerLastKnownPositionLog;
    protected LoggedTunableNumber kp;
    protected LoggedTunableNumber ki;
    protected LoggedTunableNumber kd;
    protected LoggedTunableNumber kv;
    protected LoggedTunableNumber ka;
    protected LoggedTunableNumber ks;
    protected LoggedTunableNumber maxVelocity;
    protected LoggedTunableNumber maxAcceleration;

    // SysID
    Map<String, MotorData> motorData = Map.of(
            "roller_motor", new MotorData(
                    rollerData.rollerAppliedVolts,
                    rollerData.rollerPositionRad,
                    rollerData.rollerVelocityRadPerSec,
                    0));

    private SysIdRoutine.Config config = new SysIdRoutine.Config(
        Volts.per(Seconds).of(1), // Voltage ramp rate
        Volts.of(7), // Max voltage
        Seconds.of(4) // Test duration
    );

    private SysIdTuner sysIdTuner;

    public Roller(Implementations implementation, SimpleMotorFeedforward rollerFF) {
        rollerIO = Robot.isSimulation() ? new RollerSim(implementation)
                : new RollerSparkMax(implementation);

        String name = implementation.name();
        this.rollerFF = rollerFF;
        this.rollerState = RollerConstants.RollerStates.STOP;

        rollerVelocityLog = new LoggedTunableNumber(getName() + "/" + name + " Velocity", 0.0);
        rollerVoltageLog = new LoggedTunableNumber(getName() + "/" + name + " Voltage", 0.0);
        rollerCurrentLog = new LoggedTunableNumber(getName() + "/" + name + " Current", 0.0);
        stateLog = new ShuffleData<>(getName(), "State", RollerStates.STOP.name());
        rollerPositionLog = new LoggedTunableNumber(getName() + "/" + name + " Position", 0.0);
        rollerLastKnownPositionLog = new LoggedTunableNumber(getName() + "/" + name + " Last Known Position", 0.0);

        sysIdTuner = new SysIdTuner("roller " + name, config, this, rollerIO::setVoltage, motorData);
    }

    public SysIdTuner getSysIdTuner(){
        return sysIdTuner;
    }

    public RollerIO getRollerIO() {
        return rollerIO;
    }

    public void setVoltage(double volts) {
        rollerIO.setVoltage(volts);
    }

    public void setVelocity(double velocityRadPerSec) {
        rollerIO.setVelocity(velocityRadPerSec, rollerFF.calculate(velocityRadPerSec));
    }

    public RollerStates getState() {
        return rollerState;
    }

    public void setState(RollerStates rollerState) {
        this.rollerState = rollerState;
        if (rollerState == RollerConstants.RollerStates.MAINTAIN) {
            lastKnownPosition = rollerData.rollerPositionRad;
        }
    }

    public void runRollerStates() {
        switch (rollerState) {
            case RUN:
                run();
                break;
            case MAINTAIN:
                maintain();
                break;
            case STOP:
                stop();
                break;
        }
    }

    public abstract void run();

    public void maintain() {

        rollerIO.setPosition(rollerData.rollerPositionRad, lastKnownPosition);

    }

    public void stop() {
        rollerIO.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        rollerIO.updateData(rollerData);
        runRollerStates();

        rollerVelocityLog.set(rollerData.rollerVelocityRadPerSec);
        rollerVoltageLog.set(rollerData.rollerAppliedVolts);
        rollerCurrentLog.set(rollerData.currentAmps);
        rollerPositionLog.set(rollerData.rollerPositionRad);
        rollerLastKnownPositionLog.set(lastKnownPosition);
        stateLog.set(rollerState.name());
    }

}
