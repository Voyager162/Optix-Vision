package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.PhotoelectricIO.PhotoelectricData;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.sim.PhotoelectricSim;

/**
 * Coral implementation of the roller subsystem
 *
 * @author Lilian Wu
 */

@SuppressWarnings("unused")
public class CoralRoller extends Roller {
    private double lastVelocity = 0.0;
    private boolean hasPiece = true;
    private PhotoelectricIO photoelectricIO;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private RollerData rollerData = new RollerData();
    private boolean routineStarted = false;
    private double intakeStartTime = 0;

    public CoralRoller() {
        super(Implementations.CORAL, FF(), positionPID(), velocityPID());
        photoelectricIO = new PhotoelectricSim();
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Coral.kSVelocity.get(),
                RollerConstants.Coral.kVVelocity.get(),
                RollerConstants.Coral.kAVelocity.get());
    }

    public static PIDController positionPID() {
        return new PIDController(RollerConstants.Coral.kPPosition.get(), RollerConstants.Coral.kIPosition.get(),
                RollerConstants.Coral.kDPosition.get());
    }

    public static PIDController velocityPID() {
        return new PIDController(RollerConstants.Coral.kPVelocity.get(), RollerConstants.Coral.kIVelocity.get(),
                RollerConstants.Coral.kDVelocity.get());
    }

    public boolean hasPiece() {
        SmartDashboard.putNumber("time since intake", Timer.getFPGATimestamp() - intakeStartTime);

        if (Robot.isSimulation()) {
            return photoelectricData.sensing;
        } else {
            if (!super.getIsStableState() && getState() == RollerStates.INTAKE
                    && Timer.getFPGATimestamp() - intakeStartTime > 0.45) {
                hasPiece = true;
            } else {
                hasPiece = false;
            }
            return hasPiece;
        }
    }

    @Override
    public void setState(RollerStates rollerState) {
        System.out.println(rollerState.name());
        super.setState(rollerState);
        if (rollerState.equals(RollerStates.INTAKE)) {
            intakeStartTime = Timer.getFPGATimestamp();
        }

    }

    public boolean getHasPiece() {
        return hasPiece;
    }

    /**
     * Implemetation of run method
     */
    @Override
    public void intake() {
        setVelocity(RollerStates.INTAKE.coralVelocity);
    }

    /**
     * Implemetation of run method
     */
    @Override
    public void outtake() {
        setVelocity(RollerStates.OUTTAKE.coralVelocity);
    }

    @Override
    public void maintain() {
        setVoltage(1);
    }

    @Override
    public void periodic() {
        super.periodic();
        photoelectricIO.updateData(photoelectricData);

        Logger.recordOutput("subsystems/roller/CoralRoller/hasPiece", hasPiece());
        Logger.recordOutput("subsystems/roller/CoralRoller/setInitalState", routineStarted);
        Logger.recordOutput("subsystems/roller/CoralRoller/is stable state", super.getIsStableState());

        // routineStarted is true when the routine begins in Autos
        if (Autos.isRoutineStarted() && !routineStarted) {
            routineStarted = true;
            // sets initial state at the start of each routine
            photoelectricIO.setInitialState(true);
        }

        // routineStarted is false when the routine ends in Autos
        if (!Autos.isRoutineStarted() && routineStarted) {
            routineStarted = false;
        }

        if (this.getCurrentCommand() != null) {
            SmartDashboard.putString("coral roller command", this.getCurrentCommand().getName());
        } else {
            SmartDashboard.putString("coral roller command", "null");
        }
    }
}
