package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralArmConstants;

/*
 * Handoff command for coral intake to chute
 * 
 * @author Dhyan Soni
 */
public class Handoff extends Command {

    public Handoff() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralArm.setState(CoralArmConstants.ArmStates.HAND_OFF);
        Robot.coralRoller.setState(RollerConstants.RollerStates.OUTTAKE);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.INTAKE);
    }

    @Override
    public void execute() {
        // if (Robot.coralArm.getState() == CoralArmConstants.ArmStates.HAND_OFF && Robot.coralArm.getIsStableState()
        //         && Robot.elevator.getState() == ElevatorStates.STOW && Robot.elevator.getIsStableState()) { 
        //     Robot.coralRoller.setState(RollerConstants.RollerStates.OUTTAKE); 
        // }

    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOWED);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.scoringRoller.setState(RollerStates.MAINTAIN);
    }

    /** 
     * Command finishes when scoringRoller has coral and command is being scheduled
     */
    @Override
    public boolean isFinished() {
        return Robot.scoringRoller.hasPiece() && this.isScheduled();
    }
}
