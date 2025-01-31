package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;

public class KnockAlgae extends Command {
    private final ElevatorStates state;

    public KnockAlgae(ElevatorStates state) {
        this.state = state;
        addRequirements(Robot.getAllSuperStructureSubsystems());

    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        Robot.algaeRoller.setState(RollerConstants.RollerStates.RUN);
        Robot.elevator.setState(state);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.algaeRoller.setState(RollerConstants.RollerStates.STOP);
        System.out.println("knock algae end");
    }

    @Override
    public boolean isFinished() {
        return Robot.elevator.getIsStableState(); 
    }
}
