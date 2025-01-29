package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class KnockAlgae extends Command {
    private final ElevatorStates state;

    public KnockAlgae(ElevatorStates state) {
        this.state = state;
        addRequirements(Robot.getAllSuperStructureSubsystems());

    }

    @Override
    public void initialize() {
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.STOP);
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
        System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        return Robot.elevator.getIsStableState(); // change later to isAlgaeRemoved()
    }
}
