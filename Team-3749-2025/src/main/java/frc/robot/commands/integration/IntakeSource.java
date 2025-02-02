package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
/*
 * IntakeSource command for intaking coral from source
 */
public class IntakeSource extends Command {

    public IntakeSource() {
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        if (Robot.scoringRoller.hasPiece()) {
            this.cancel();
        }
        Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        Robot.elevator.setState(ElevatorStates.SOURCE);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.RUN); 
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN);
    }

    @Override
    public boolean isFinished() {
        return Robot.scoringRoller.hasPiece();
    }
}
