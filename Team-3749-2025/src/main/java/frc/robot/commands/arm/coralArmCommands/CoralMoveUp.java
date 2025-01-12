package frc.robot.commands.arm.coralArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.coralArmConstants.ArmStates;

public class CoralMoveUp extends Command {

    public CoralMoveUp() {
        super.addRequirements(Robot.coralArm);
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(ArmStates.MOVING_UP);
    }

        @Override
    public void end(boolean interupted) {
        Robot.coralArm.setState(ArmStates.STOPPED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
