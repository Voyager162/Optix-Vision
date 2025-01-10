package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.example.ExampleSubsystemConstants.SubsystemStates;

/**
 * An example subsystem command
 * 
 * @author Noah Simon
 */
public class elevatorstop extends Command {
    public elevatorstop() {
        super.addRequirements(Robot.elevator);
    }

    @Override
    public void initialize() {
    }

    // if further state change are needed over time, put them here with conditionals
    @Override
    public void execute() {
        if (Robot.subsystem.getIsStableState()) {
            Robot.subsystem.setState(SubsystemStates.STOP);
        }
        Robot.elevator.setVoltage(0);;
    }

    @Override
    public void end(boolean interupted) {
        Robot.subsystem.setState(SubsystemStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
