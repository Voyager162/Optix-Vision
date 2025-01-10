package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/**
 * Command to travel to a specific angle
 * 
 * @Author Weston Gardner
 * 
 */

public class setVoltage extends Command {

    private double voltage;

    public setVoltage(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public void initialize() {
        Robot.arm.setVoltage(voltage);
    }

    @Override
    public void execute() {}


    
}
