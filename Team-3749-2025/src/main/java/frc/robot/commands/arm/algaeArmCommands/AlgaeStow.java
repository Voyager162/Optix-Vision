package frc.robot.commands.arm.algaeArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.algaeArmConstants;
import frc.robot.subsystems.arm.ArmConstants.algaeArmConstants.ArmStates;
import frc.robot.utils.UtilityFunctions;

public class AlgaeStow extends Command {

    public AlgaeStow() {
        super.addRequirements(Robot.algaeArm);
    }

    @Override
    public void initialize() {
        Robot.algaeArm.setState(ArmStates.STOWED);
    }

        @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        double position = Robot.algaeArm.getPositionRad();
        return UtilityFunctions.withinMargin(0.001, algaeArmConstants.stowSetPoint_rad, position);
    }
}
