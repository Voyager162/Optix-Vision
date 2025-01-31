package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.algae.AlgaeConstants.ArmStates;
import frc.robot.subsystems.arm.coral.CoralConstants;

public class OuttakeCoral extends Command {

    public OuttakeCoral() {
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralConstants.ArmStates.CORAL_PICKUP);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.elevator.setState(ElevatorStates.STOW);
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralConstants.ArmStates.CORAL_PICKUP && Robot.coralArm.getIsStableState()) {
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        System.out.println("outtake coral: " + Robot.coralRoller.hasPiece());
        return !Robot.coralRoller.hasPiece() && !Robot.coralRoller.getIsStableState();
    }
}


