package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;

public class Stow extends Command {

    public Stow() {
        super.addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {
        Robot.arm.setState(ArmStates.STOWED);
    }

        @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        double position = Robot.arm.getPositionRad();
        return position > 1.58 && position < 1.57;
    }
}
