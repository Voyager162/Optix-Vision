package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;

public class KnockAlgae extends Command{   
    private final Elevator elevator;
    private final Roller algaeRoller;
    private final ElevatorStates state;

    public KnockAlgae(Elevator elevator, Roller algaeRoller, ElevatorStates state) {
        this.elevator = elevator;
        this.algaeRoller = algaeRoller;
        this.state = state;
    }

    @Override
    public void initialize() {
        algaeRoller.setState(RollerConstants.RollerStates.MAINTAIN); 
    }

    @Override 
    public void execute() {
        algaeRoller.setState(RollerConstants.RollerStates.RUN);
        elevator.setState(state);
        System.out.println(elevator.getState());
    }

    @Override 
    public void end(boolean interrupted) {
        elevator.setState(ElevatorStates.STOW);
        algaeRoller.setState(RollerConstants.RollerStates.STOP); 
    }

    @Override
    public boolean isFinished() {
        return elevator.getIsStableState(); // change later to isAlgaeRemoved()
    }
}
