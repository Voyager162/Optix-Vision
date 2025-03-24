package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants.ArmStates;

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
        Robot.scoringRoller.setHandoffComplete(false);

        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralArm.setState(CoralArmConstants.ArmStates.HAND_OFF);
        Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.INTAKE);
    }

    @Override
    public void execute() {
        System.out.println("Handoff");
        if (Robot.coralArm.getIsStableState() && Robot.coralArm.getState() == ArmStates.HAND_OFF) {
            Robot.coralRoller.setState(RollerConstants.RollerStates.OUTTAKE);

        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOW);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.scoringRoller.setState(RollerStates.MAINTAIN);
        if (interrupted == false) {

            Robot.scoringRoller.setHandoffComplete(true);
        }

    }

    /**
     * Command finishes when scoringRoller has coral and command is being scheduled
     */
    @Override
    public boolean isFinished() {
        return Robot.scoringRoller.hasPiece() && this.isScheduled();
    }
}
