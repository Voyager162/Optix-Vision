package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * Command to travel to a specific angle
 * 
 * @Author Weston Gardner
 * 
 */

public class Move extends Command {

    private double voltage;

    public Move(double voltage) {
        this.voltage = voltage;
        addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Robot.arm.setCommandedVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.arm.setCommandedVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
