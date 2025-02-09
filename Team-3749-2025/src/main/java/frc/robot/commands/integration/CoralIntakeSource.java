package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

/**
 * CoralIntakeSource command for intaking coral from source
 */
public class CoralIntakeSource extends Command {

    public CoralIntakeSource() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems()); 
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.SOURCE);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.RUN);
    }    

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.HAND_OFF);
        Robot.coralRoller.setState(RollerStates.MAINTAIN);
    }

    /**
     * Command finishes when coralRoller has coral and command is being scheduled
     */
    @Override
    public boolean isFinished() {
        return Robot.coralRoller.hasPiece() && this.isScheduled();
    }
}
