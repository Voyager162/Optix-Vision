package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
/*
 * IntakeSource command for intaking coral from source
 */
public class IntakeSource extends Command {
    
    public IntakeSource() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        if (Robot.scoringRoller.hasPiece()) {
            this.cancel(); // cancels command if scoringRoller has coral initially
        }
        else {
            Robot.coralArm.setState(CoralArmConstants.ArmStates.STOWED);
            Robot.elevator.setState(ElevatorStates.SOURCE);
            Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
            Robot.scoringRoller.setState(RollerConstants.RollerStates.RUN); 
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN);
    }

    // command finishes when scoringRoller has coral and command is being scheduled
    @Override
    public boolean isFinished() {
        return Robot.scoringRoller.hasPiece() && this.isScheduled();
    }
}
