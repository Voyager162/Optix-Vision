package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;

public class MoveUp extends Command {

    public MoveUp() {
        super.addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {
        Robot.arm.setState(ArmStates.MOVING_UP);
    }

        @Override
    public void end(boolean interupted) {
        Robot.arm.setState(ArmStates.STOPPED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
