package frc.robot.commands.arm.algaeArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.algaeArmConstants.ArmStates;

public class AlgaeMoveDown extends Command {

    public AlgaeMoveDown() {
        super.addRequirements(Robot.algaeArm);
    }

    @Override
    public void initialize() {
        Robot.algaeArm.setState(ArmStates.MOVING_DOWN);
    }

        @Override
    public void end(boolean interupted) {
        Robot.algaeArm.setState(ArmStates.STOPPED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
