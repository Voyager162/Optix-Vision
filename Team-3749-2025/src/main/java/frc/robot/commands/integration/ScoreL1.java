package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

/*
 * ScoreL1 command for scoring coral on L1 using coral arm
 */
public class ScoreL1 extends Command {

    public ScoreL1() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        if (Robot.coralRoller.hasPiece()) {
            Robot.elevator.setState(ElevatorConstants.ElevatorStates.STOW);
            Robot.coralArm.setState(CoralArmConstants.ArmStates.L1);
            Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
            Robot.scoringRoller.setState(RollerConstants.RollerStates.STOP);
        }
        else {
            this.cancel();
        }
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralArmConstants.ArmStates.L1 && Robot.coralArm.getIsStableState()) {
            Robot.coralRoller.setState(RollerConstants.RollerStates.OUTTAKE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOWED);
        Robot.coralRoller.setState(RollerStates.STOP);
    }

    /** 
     * Command finishes when coralRoller does not have coral and command is being scheduled
     */
    @Override
    public boolean isFinished() {
        return !Robot.coralRoller.hasPiece() && this.isScheduled();
    }
}
