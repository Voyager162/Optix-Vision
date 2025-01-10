package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;

public class FullyExtend extends Command {

    public FullyExtend() {
        super.addRequirements(Robot.arm);
    }

    @Override
    public void initialize() {
        Robot.arm.setState(ArmStates.FULLY_EXTENDED);
    }

        @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        double position = Robot.arm.getPositionRad();
        return position > 3.141 && position < 3.142;
    }
}
