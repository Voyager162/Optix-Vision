package frc.robot.commands.arm.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.climbArmConstants.ArmStates;

public class ClimbMoveDown extends Command {

    public ClimbMoveDown() {
        super.addRequirements(Robot.climbArm);
    }

    @Override
    public void initialize() {
        Robot.climbArm.setState(ArmStates.MOVING_DOWN);
    }

        @Override
    public void end(boolean interupted) {
        Robot.climbArm.setState(ArmStates.STOPPED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
