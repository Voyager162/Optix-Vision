package frc.robot.subsystems.roller;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.real.RollerSparkMax;
import frc.robot.subsystems.roller.sim.RollerSim;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.ShuffleData;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public abstract class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerData rollerData;
    private RollerStates rollerState;
    private SimpleMotorFeedforward rollerFF;
    private double lastKnownPosition = 0.0;

    private LoggedTunableNumber rollerVelocityLog;
    private LoggedTunableNumber rollerVoltageLog;
    private LoggedTunableNumber rollerCurrentLog;
    private ShuffleData<String> stateLog;
    private LoggedTunableNumber rollerPositionLog;
    private LoggedTunableNumber rollerLastKnownPositionLog;

    public Roller(Implementations implementation, SimpleMotorFeedforward rollerFF) {
        rollerIO = Robot.isSimulation() ? new RollerSim(implementation)
                : new RollerSparkMax(implementation);

        String name = implementation.name();
        this.rollerFF = rollerFF;
        this.rollerState = RollerConstants.RollerStates.STOP;
        rollerData = new RollerData();

        rollerVelocityLog = new LoggedTunableNumber(getName() + "/" + name + " Velocity", 0.0);
        rollerVoltageLog = new LoggedTunableNumber(getName() + "/" + name + " Voltage", 0.0);
        rollerCurrentLog = new LoggedTunableNumber(getName() + "/" + name + " Current", 0.0);
        stateLog = new ShuffleData<>(getName(), "State", RollerStates.STOP.name());
        rollerPositionLog = new LoggedTunableNumber(getName() + "/" + name + " Position", 0.0);
        rollerLastKnownPositionLog = new LoggedTunableNumber(getName() + "/" + name + " Last Known Position", 0.0);
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
