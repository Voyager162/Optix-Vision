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
        
        Robot.elevator.setState(elevatorState);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralArm.setState(ArmStates.STOW);

    }

    @Override
    public void execute() {
        if (Robot.scoringRoller.hasPiece()) {
            pieceRecognized = true;
        }
        
        // scores when elevator reaches desired state
        if (Robot.elevator.getState() == elevatorState && Robot.elevator.getIsStableState()) {
            Robot.scoringRoller.setState(RollerStates.SCORE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(ArmStates.STOW);
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
