package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
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
        // if (Robot.scoringRoller.hasPiece()) {
        Robot.elevator.setState(elevatorState);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralArm.setState(ArmStates.STOWED);
        // handoff from coral arm to elevator to score
        // } else if (Robot.coralRoller.hasPiece()) {
        // Robot.coralArm.setState(ArmStates.HAND_OFF);
        // Robot.elevator.setState(ElevatorStates.STOW);
        // Robot.coralRoller.setState(RollerStates.MAINTAIN);
        // } else {
        // this.cancel(); // cancels command if neither scoringRoller nor coralRoller
        // has coral
        // }
    }

    @Override
    public void execute() {
        if (Robot.scoringRoller.hasPiece()) {
            pieceRecognized = true;
        }
        // handoff execution
        // if ((Robot.coralArm.getState() == ArmStates.HAND_OFF) &&
        // Robot.coralArm.getIsStableState() &&
        // (Robot.elevator.getState() == ElevatorStates.STOW) &&
        // Robot.elevator.getIsStableState()) {
        // Robot.coralRoller.setState(RollerStates.OUTTAKE);
        // Robot.scoringRoller.setState(RollerStates.INTAKE);
        // handoffComplete = true;
        // }
        // // checks whether handoff is complete
        // if (handoffComplete && !Robot.coralRoller.hasPiece() &&
        // Robot.scoringRoller.hasPiece()) {
        // Robot.scoringRoller.setState(RollerStates.MAINTAIN);
        // Robot.elevator.setState(elevatorState);
        // }
        // scores when elevator reaches desired state
        if (Robot.elevator.getState() == elevatorState && Robot.elevator.getIsStableState()) {
            Robot.scoringRoller.setState(RollerStates.OUTTAKE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(ArmStates.STOWED);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.STOP);
        pieceRecognized = false;
    }

    /**
     * Command finishes when scoringRoller does not have coral and command is being
     * scheduled
     */
    @Override
    public boolean isFinished() {
        return !Robot.scoringRoller.hasPiece() && pieceRecognized && this.isScheduled();
        // return false;
    }
}
