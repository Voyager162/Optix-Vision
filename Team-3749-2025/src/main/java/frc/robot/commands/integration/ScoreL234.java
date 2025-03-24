package frc.robot.commands.integration;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.utils.UtilityFunctions;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants.ArmStates;

/*
 * ScoreL234 command for scoring coral on L2, L3, L4
 */
@SuppressWarnings("unused")
public class ScoreL234 extends Command {
    private final ElevatorStates elevatorState;
    private boolean handoffComplete = false;
    private boolean pieceRecognized = false;
    private double scoreTimestamp = Double.MAX_VALUE;
    private double cmdStartTimestamp = Double.MAX_VALUE;

    /**
     * 
     * @param elevatorState
     */
    public ScoreL234(ElevatorStates elevatorState) {
        this.elevatorState = elevatorState;
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(elevatorState);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralArm.setState(ArmStates.STOW);
        // Robot.vision.setCameraStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, 1);
        // Robot.vision.setCameraStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, 2);
        cmdStartTimestamp = Timer.getFPGATimestamp();

    }

    @Override
    public void execute() {
        if (Robot.scoringRoller.hasPiece()) {
            pieceRecognized = true;
        }
        Logger.recordOutput("swerve at setpoint", Robot.swerve.atSwerveSetpoint(Robot.swerve.getPPSetpoint().setpoint));
        // scores when elevator reaches desired state
        if (Robot.elevator.getState() == elevatorState && Robot.elevator.getIsStableState()
                && scoreTimestamp == Double.MAX_VALUE) {
            boolean otfReachedSetpoint = (Robot.swerve.getIsOTF()
                    && Robot.swerve.atSwerveSetpoint(Robot.swerve.getPPSetpoint().setpoint));
            boolean autoReachedSetpoint = ((DriverStation.isAutonomous()
                    && Robot.swerve.atSwerveSetpoint(Robot.swerve.getPositionSetpoint())));

            if ((otfReachedSetpoint
                    || autoReachedSetpoint)) {

                Robot.scoringRoller.setState(RollerStates.SCORE);
                return;
            } else if ((Timer.getFPGATimestamp() - cmdStartTimestamp < 3.5)) {
                Robot.scoringRoller.setState(RollerStates.SCORE);
            } else {
                Robot.scoringRoller.setState(RollerStates.STOP);
            }

            if (scoreTimestamp == Double.MAX_VALUE) {
                scoreTimestamp = Timer.getFPGATimestamp();
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(ArmStates.STOW);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.STOP);
        // Robot.vision.setCameraStrategy(PoseStrategy.CONSTRAINED_SOLVEPNP, 1);
        // Robot.vision.setCameraStrategy(PoseStrategy.CONSTRAINED_SOLVEPNP, 2);

        pieceRecognized = false;
        scoreTimestamp = Double.MAX_VALUE;
        cmdStartTimestamp = Double.MAX_VALUE;
        if (interrupted == false) {

            Robot.scoringRoller.setHandoffComplete(false);
        }
    }

    /**
     * Command finishes when scoringRoller does not have coral and command is being
     * scheduled
     */
    @Override
    public boolean isFinished() {
        return !Robot.scoringRoller.hasPiece() && pieceRecognized &&
                Timer.getFPGATimestamp() - scoreTimestamp > 1.5 && this.isScheduled(); // was 0.9
        // return Timer.getFPGATimestamp() - scoreTimestamp > 1 && this.isScheduled();
        // return false;
    }

}
