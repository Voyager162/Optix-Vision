package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

/*
 * ScoreL234 command for scoring coral on L2, L3, L4
 */
public class ScoreL234 extends Command {
    private final ElevatorStates elevatorState;
    private boolean handoffComplete = false;

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
        if (Robot.scoringRoller.hasPiece()) {
            Robot.elevator.setState(elevatorState);
            Robot.scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN);
            Robot.coralArm.setState(CoralArmConstants.ArmStates.STOWED);
        // handoff from coral arm to elevator to score
        } else if (Robot.coralRoller.hasPiece()) {
            Robot.coralArm.setState(CoralArmConstants.ArmStates.HAND_OFF);
            Robot.elevator.setState(ElevatorStates.STOW);
            Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
        } else {
            this.cancel(); // cancels command if neither scoringRoller nor coralRoller has coral
        }
    }

    @Override
    public void execute() {
        // handoff execution
        if ((Robot.coralArm.getState() == CoralArmConstants.ArmStates.HAND_OFF) && Robot.coralArm.getIsStableState() &&
            (Robot.elevator.getState() == ElevatorStates.STOW) && Robot.elevator.getIsStableState()) {
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE);
            Robot.scoringRoller.setState(RollerConstants.RollerStates.RUN);
            handoffComplete = true;
        }
        // checks whether handoff is complete
        if (handoffComplete && !Robot.coralRoller.hasPiece() && Robot.scoringRoller.hasPiece()) {
            Robot.scoringRoller.setState(RollerStates.MAINTAIN);
            Robot.elevator.setState(elevatorState);
        }
        // scores when elevator reaches desired state
        if (Robot.elevator.getState() == elevatorState && Robot.elevator.getIsStableState()) {
            Robot.scoringRoller.setState(RollerConstants.RollerStates.SCORE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOWED);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
    }

    // command finishes when scoringRoller does not have coral and command is being scheduled
    @Override
    public boolean isFinished() {
        return !Robot.scoringRoller.hasPiece() && this.isScheduled();
    }
}
