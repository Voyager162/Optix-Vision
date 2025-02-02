package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
/*
 * ScoreL234 command for scoring coral on L2, L3, L4
 */
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
            System.out.println("scoreL234 start");
        } else if (Robot.coralRoller.hasPiece()) {
            Robot.coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
            Robot.elevator.setState(ElevatorStates.STOW);
            Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN); 
        } else {
            System.out.println("Canceling");
            this.cancel();
        }
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralConstants.ArmStates.HAND_OFF && Robot.coralArm.getIsStableState() 
                && Robot.elevator.getState() == ElevatorStates.STOW && Robot.elevator.getIsStableState()) { 
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE); 
            Robot.scoringRoller.setState(RollerConstants.RollerStates.RUN);
            handoffComplete = true;
            Robot.scoringRoller.setHasPiece(false);
        }

        if (handoffComplete && !Robot.coralRoller.hasPiece() && Robot.scoringRoller.hasPiece()) { 
            Robot.scoringRoller.setState(RollerStates.MAINTAIN);
            Robot.elevator.setState(state);
            Robot.scoringRoller.setHasPiece(false);
        }
    
        if (Robot.elevator.getState() == state && Robot.elevator.getIsStableState()) {
            Robot.scoringRoller.setState(RollerConstants.RollerStates.SCORE);
            Robot.scoringRoller.setHasPiece(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        System.out.println("scoreL234 end");
    }

    @Override
    public boolean isFinished() {
        System.out.println("scoreL234 finishing: " + Robot.scoringRoller.hasPiece());
        return !Robot.scoringRoller.hasPiece();
    }
}
