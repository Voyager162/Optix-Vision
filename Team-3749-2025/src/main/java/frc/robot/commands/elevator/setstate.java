package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.example.ExampleSubsystemConstants.SubsystemStates;

/**
 * An example subsystem command
 * 
 * @author Noah Simon
 */
public class setstate extends Command {
    ElevatorStates states;
    public setstate(ElevatorStates states) {
        this.states = states;
        super.addRequirements(Robot.elevator);
    }

    @Override
    public void initialize() {
    }

    // if further state change are needed over time, put them here with conditionals
    @Override
    public void execute() {
        /*if (Robot.elevator.getIsStableState()) {
            Robot.elevator.setState(ElevatorStates.STOP);
        }*/
        Robot.elevator.setState(states);
    }

    @Override
    public void end(boolean interupted) {
        Robot.elevator.setState(ElevatorStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
