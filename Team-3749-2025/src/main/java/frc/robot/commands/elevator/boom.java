package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

/**
 * Elevator states command
 * 
 * @author Dhyan Soni
 * @author Andrew Ge
 */
public class boom extends Command {
    double volts;
    public boom(double volts) {
        this.volts = volts;
        super.addRequirements(Robot.elevator);
    }

    @Override
    public void initialize() {
        Robot.elevator.setVoltage(volts);
    }

    // if further state change are needed over time, put them here with conditionals
    @Override
    public void execute() {
        /*if (Robot.elevator.getIsStableState()) {
            Robot.elevator.setState(ElevatorStates.STOP);
        }*/
    }

    @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
