package frc.robot.commands.arm.algeaArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.algeaArmConstants;
import frc.robot.subsystems.arm.ArmConstants.algeaArmConstants.ArmStates;
import frc.robot.utils.UtilityFunctions;

public class AlgeaStow extends Command {

    public AlgeaStow() {
        super.addRequirements(Robot.algeaArm);
    }

    @Override
    public void initialize() {
        Robot.algeaArm.setState(ArmStates.STOWED);
    }

        @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        double position = Robot.algeaArm.getPositionRad();
        return UtilityFunctions.withinMargin(0.001, algeaArmConstants.stowSetPoint_rad, position);
    }
}
