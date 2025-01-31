package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
//works for l2-4 change name
public class ScoreL234 extends Command {
    private final ElevatorStates state;
    private boolean handoffComplete = false;

    public ScoreL234(ElevatorStates state) {
        this.state = state;
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        if (Robot.scoringRoller.hasPiece()) { 
            Robot.elevator.setState(state);
            Robot.scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN);
            Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        } else if (Robot.coralRoller.hasPiece()) {
            Robot.coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
            Robot.elevator.setState(ElevatorStates.STOW);
            Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN); 
        } else {
            this.cancel();
        }
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralConstants.ArmStates.HAND_OFF && Robot.coralArm.getIsStableState() 
                && Robot.elevator.getState() == ElevatorStates.STOW) { // add working elevator isStableState later
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE); 
            Robot.scoringRoller.setState(RollerConstants.RollerStates.RUN);
            handoffComplete = true;
        }
        if (handoffComplete && !Robot.coralRoller.hasPiece() && Robot.scoringRoller.hasPiece()) { // should work once hasPiece logic is made
            Robot.scoringRoller.setState(RollerStates.MAINTAIN);
            Robot.elevator.setState(state);
        }
        if (Robot.elevator.getState() == state && Robot.elevator.getIsStableState()) {
            Robot.scoringRoller.setState(RollerConstants.RollerStates.SCORE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralConstants.ArmStates.STOPPED);
        Robot.elevator.setState(ElevatorStates.STOP);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        System.out.println("ScoreL234: " + Robot.coralRoller.hasPiece());
        return !Robot.scoringRoller.hasPiece();
    }
}
