package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.buttons.JoystickIO;
import frc.robot.buttons.ButtonBoard.ScoringMode;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralArmConstants;

/*
 * Handoff command for coral intake to chute
 * 
 * @author Dhyan Soni
 */
public class ScoringModeConditionalHandoff extends Command {

    public ScoringModeConditionalHandoff() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        if (JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.L1 ||JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.ALGAE ){
            cancel();
            return;
        }
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralArm.setState(CoralArmConstants.ArmStates.HAND_OFF);
        Robot.coralRoller.setState(RollerConstants.RollerStates.OUTTAKE);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.INTAKE);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOW);
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
