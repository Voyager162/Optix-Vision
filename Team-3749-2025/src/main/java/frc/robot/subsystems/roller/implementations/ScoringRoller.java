package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.PhotoelectricIO.PhotoelectricData;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.real.JTVisiSight;
import frc.robot.subsystems.roller.sim.PhotoelectricSim;

/**
 * Scoring implementation of the roller subsystem
 *
 * @author Lilian Wu
 */
public class ScoringRoller extends Roller {
    private RollerData rollerData;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private PhotoelectricIO photoelectricIO;
    private boolean hasPiece = true;
    private boolean routineStarted = false;
    private boolean isAlgaeMode = false;
    // this *should* just be has piece, but it's too close to comp to make neat rn
    private boolean handoffComplete = true;

    public ScoringRoller() {
        super(Implementations.SCORING, FF(), positionPID(), velocityPID());
        this.rollerData = new RollerData();
        if (Robot.isSimulation()) {
            this.photoelectricIO = new PhotoelectricSim();
            photoelectricIO.setInitialState(true);
        } else {
            this.photoelectricIO = new JTVisiSight();
        }
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Scoring.kSVelocity.get(),
                RollerConstants.Scoring.kVVelocity.get(),
                RollerConstants.Scoring.kAVelocity.get());
    }

    public static PIDController positionPID() {
        return new PIDController(RollerConstants.Scoring.kPPosition.get(), RollerConstants.Scoring.kIPosition.get(),
                RollerConstants.Scoring.kDPosition.get());
    }

    public static PIDController velocityPID() {
        return new PIDController(RollerConstants.Scoring.kPVelocity.get(), RollerConstants.Scoring.kIVelocity.get(),
                RollerConstants.Scoring.kDVelocity.get());
    }

    public boolean getIsAlgaeMode() {
        return isAlgaeMode;
    }

    public void setIsAlgaeMode(boolean isAlgaeMode) {
        this.isAlgaeMode = isAlgaeMode;
    }

    @Override
    public void outtake() {
        if (isAlgaeMode) {
            setVelocity(RollerStates.OUTTAKE.algaeVelocity);
            return;
        }
        setVelocity(RollerStates.OUTTAKE.scoringVelocity);
    }

    @Override
    public void score() {
        setVelocity(RollerStates.SCORE.scoringVelocity);
    }

    public static PIDController positionController() {
        return new PIDController(RollerConstants.Scoring.kPPosition.get(), RollerConstants.Scoring.kIPosition.get(),
                RollerConstants.Scoring.kDPosition.get());
    }

    /**
     * Implemetation of run method
     */
    @Override
    public void intake() {
        if (isAlgaeMode) {
            setVelocity(RollerConstants.RollerStates.INTAKE.algaeVelocity);
            return;
        }
        if (!rollerData.sensorTripped) {
            setVelocity(RollerConstants.RollerStates.INTAKE.scoringVelocity);
            return;
        }
        setVoltage(0.0);
    }

    public boolean hasPiece() {
        return hasPiece;
    }

    public boolean getHandoffComplete() {
        return handoffComplete;
    }

    public void setHandoffComplete(boolean handoffComplete) {
        this.handoffComplete = handoffComplete;
    }

    int resetPiece = 0;

    public void setHasPiece(boolean hasPiece) {
        SmartDashboard.putNumber("reset piece", resetPiece);
        resetPiece++;

        SmartDashboard.putBoolean("requested has pice", hasPiece);

        this.hasPiece = hasPiece;
    }

    @Override
    public void maintain() {
        Logger.recordOutput("Roller/ScoringRoller/setpointPosition",
                getLastKnownPosition() - RollerConstants.Scoring.reverseDistance);
        setPosition(getLastKnownPosition() - RollerConstants.Scoring.reverseDistance,
                RollerConstants.Scoring.kSVelocity.get());
    }

    @Override
    public void periodic() {
        super.periodic();

        photoelectricIO.updateData(photoelectricData);

        hasPiece = photoelectricData.sensing;

        Logger.recordOutput("Roller/ScoringRoller/hasPiece", hasPiece);
        Logger.recordOutput("Roller/ScoringRoller/setInitalState", routineStarted);
        Logger.recordOutput("Roller/ScoringRoller/isAlgaeMode", isAlgaeMode);

        // routineStarted is true when the routine begins in Autos
        if (Autos.isRoutineStarted() && !routineStarted) {
            routineStarted = true;
            // routineStarted is true when the routine begins in Autos
            if (Autos.isRoutineStarted() && !routineStarted) {
                routineStarted = true;
                // sets initial state at the start of each routine
                photoelectricIO.setInitialState(true);
                photoelectricIO.setInitialState(true);
            }

            // routineStarted is false when the routine ends in Autos

            // routineStarted is false when the routine ends in Autos
            if (!Autos.isRoutineStarted() && routineStarted) {
                routineStarted = false;
                routineStarted = false;
            }

            if (this.getCurrentCommand() != null) {
                SmartDashboard.putString("scoring roller command", this.getCurrentCommand().getName());
                return; // if you add more stuff here then this will cause problems
            }
        }
    }
}
