package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;

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
        Robot.arm.setState(ArmStates.MOVING);
    }

    @Override
    public void execute() {
        Robot.arm.setVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.arm.setVoltage(0);
        Robot.arm.setState(ArmStates.STOPPED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
