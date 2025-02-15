package frc.robot.subsystems.roller;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.real.RollerSparkMax;
import frc.robot.subsystems.roller.sim.RollerSim;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.MotorData;
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

    protected LoggedTunableNumber kv;
    protected LoggedTunableNumber ka;
    protected LoggedTunableNumber ks;
    protected LoggedTunableNumber maxVelocity;
    protected LoggedTunableNumber maxAcceleration;

    


    public Roller(Implementations implementation, SimpleMotorFeedforward rollerFF) {
        rollerIO = Robot.isSimulation() ? new RollerSim(implementation)
                : new RollerSparkMax(implementation);

        String name = implementation.name();
        this.rollerFF = rollerFF;
        this.rollerState = RollerConstants.RollerStates.STOP;

    }

    public RollerIO getRollerIO() {
        return rollerIO;
    }

    public void setVoltage(double volts) {
        System.out.println(volts);
        rollerIO.setVoltage(volts);
    }

    public void setVelocity(double velocityRadPerSec) {
        rollerIO.setVelocity(velocityRadPerSec, rollerFF.calculate(velocityRadPerSec));
    }

    public RollerStates getState() {
        return rollerState;
    }

    /**
     * Sets the roller state
     * 
     * Stores last position when the state is maintain
     */
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

    /**
     * Maintains the roller's last known position
     */
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

        Logger.recordOutput("subsystems/roller/" + getName() + "/velocity", rollerData.rollerVelocityRadPerSec);
        Logger.recordOutput("subsystems/roller/" + getName() + "/applied voltage", rollerData.rollerAppliedVolts);
        Logger.recordOutput("subsystems/roller/" + getName() + "/current", rollerData.currentAmps);
        Logger.recordOutput("subsystems/roller/" + getName() + "/position", rollerData.rollerPositionRad);
        Logger.recordOutput("subsystems/roller/" + getName() + "/last known position", lastKnownPosition);
        Logger.recordOutput("subsystems/roller/" + getName() + "/state", rollerState.name());
    }

}
