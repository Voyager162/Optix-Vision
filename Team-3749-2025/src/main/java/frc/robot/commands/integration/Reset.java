package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;

/*
 * Handoff command for coral intake to chute
 * 
 * @author Dhyan Soni
 */
public class Reset extends Command {

    public Reset() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOW);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.climbArm.setState(ClimbArmConstants.ArmStates.STOWED);
        Robot.swerve.setIsOTF(false);
        Robot.coralRoller.setHasPiece(false);
    }

    @Override
    public void execute() {


    }

    @Override
    public void end(boolean interrupted) {

    }

    /** 
     * Command finishes when scoringRoller has coral and command is being scheduled
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
