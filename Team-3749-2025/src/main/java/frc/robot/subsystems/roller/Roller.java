package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.implementations.ScoringRoller;
import frc.robot.subsystems.roller.real.RollerSparkMax;
import frc.robot.subsystems.roller.sim.RollerSim;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.UtilityFunctions;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public abstract class Roller extends SubsystemBase {
    private RollerIO rollerIO;
    private RollerData rollerData = new RollerData();
    private RollerStates rollerState;
    private Implementations implementation;
    private SimpleMotorFeedforward rollerFF;
    @SuppressWarnings("unused")
    private PIDController positionController;
    private PIDController velocityController;

    private double lastKnownPosition = 0.0;

    protected LoggedTunableNumber kv;
    protected LoggedTunableNumber ka;
    protected LoggedTunableNumber ks;
    protected LoggedTunableNumber maxVelocity;
    protected LoggedTunableNumber maxAcceleration;

    public Roller(Implementations implementation, SimpleMotorFeedforward rollerFF, PIDController positionController,
            PIDController velocityController) {
        rollerIO = Robot.isSimulation() ? new RollerSim(implementation)
                : new RollerSparkMax(implementation);
        this.implementation = implementation;

        // String name = implementation.name();
        this.rollerFF = rollerFF;
        this.positionController = positionController;
        this.velocityController = velocityController;
        this.rollerState = RollerConstants.RollerStates.STOP;
    }

    public RollerIO getRollerIO() {
        return rollerIO;
    }

    public void setVoltage(double volts) {
        // System.out.println(volts);
        rollerIO.setVoltage(volts);
    }

    public void setVelocity(double velocityRadPerSec) {
        double PIDOutput = velocityController.calculate(rollerData.rollerVelocityRadPerSec, velocityRadPerSec);
        double FFOutput = rollerFF.calculate(velocityRadPerSec);
        rollerIO.setVoltage(PIDOutput + FFOutput);
        Logger.recordOutput("subsystems/roller/" + getName() + "/pid", PIDOutput);
        Logger.recordOutput("subsystems/roller/" + getName() + "/ff", FFOutput);
        // rollerIO.setVelocity(velocityRadPerSec,
        // rollerFF.calculate(velocityRadPerSec));
    }

    public void setPosition(double position, double kS) {
        double PIDOutput = positionController.calculate(rollerData.rollerPositionRad, position);
        Logger.recordOutput("subsystems/roller/" + getName() + "/position pid", PIDOutput);
        double volts = PIDOutput + Math.copySign(kS, PIDOutput);
        rollerIO.setVoltage(volts);
    }

    public RollerStates getState() {
        return rollerState;
    }

    public double getLastKnownPosition() {
        return lastKnownPosition;
    }

    public double getPosition() {
        return rollerData.rollerPositionRad;
    }

    /**
     * Sets the roller state
     * 
     * Stores last position when the state is maintain
     */
    public void setState(RollerStates rollerState) {
        // System.out.println(rollerState.name());
        this.rollerState = rollerState;
        if (rollerState == RollerConstants.RollerStates.MAINTAIN) {
            setLastKnownPosition(rollerData.rollerPositionRad);
        }
    }

    public void setLastKnownPosition(double pos) {
        lastKnownPosition = pos;
    }

    public void runRollerStates() {
        switch (rollerState) {
            case INTAKE:
                intake();
                break;
            case MAINTAIN:
                maintain();
                break;
            case STOP:
                stop();
                break;
            case OUTTAKE:
                outtake();
                break;
        }
    }

    public abstract void outtake();
    public abstract void intake();

    public abstract void maintain();

    public void stop() {
        rollerIO.setVoltage(0.0);
    }

    public boolean getIsStableState() {
        switch (implementation) {
            case CORAL:
                return UtilityFunctions.withinMargin(4, rollerData.rollerVelocityRadPerSec, rollerState.coralVelocity);
            case ALGAE:
                return UtilityFunctions.withinMargin(4, rollerData.rollerVelocityRadPerSec, rollerState.algaeVelocity);
            case SCORING:
                return UtilityFunctions.withinMargin(4, rollerData.rollerVelocityRadPerSec,
                        rollerState.scoringVelocity);
            default:
                return false;
        }
    }

    @Override
    public void periodic() {

        velocityController = new PIDController(RollerConstants.Coral.kPVelocity.get(), RollerConstants.Coral.kIVelocity.get(), RollerConstants.Coral.kDVelocity.get());
        rollerIO.updateData(rollerData);
        runRollerStates();

        Logger.recordOutput("subsystems/roller/" + getName() + "/velocity", rollerData.rollerVelocityRadPerSec);
        Logger.recordOutput("subsystems/roller/" + getName() + "/applied voltage", rollerData.rollerAppliedVolts);
        Logger.recordOutput("subsystems/roller/" + getName() + "/current", rollerData.currentAmps);
        Logger.recordOutput("subsystems/roller/" + getName() + "/position", rollerData.rollerPositionRad);
        Logger.recordOutput("subsystems/roller/" + getName() + "/last known position", lastKnownPosition);
        Logger.recordOutput("subsystems/roller/" + getName() + "/state", rollerState.name());
        Logger.recordOutput("subsystems/roller/" + getName() + "/acceleration", rollerData.acceleration);
        Logger.recordOutput("subsystems/roller/" + getName() + "/stable state", getIsStableState());
        Logger.recordOutput("subsystems/roller/" + this.getName() + "/position controller values",
                positionController.getP() + " | " + positionController.getD());
    }
}