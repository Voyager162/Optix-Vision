package frc.robot.commands.arm.coralArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.coralArmConstants;
import frc.robot.subsystems.arm.ArmConstants.coralArmConstants.ArmStates;
import frc.robot.utils.UtilityFunctions;

public class HandOff extends Command {

    public HandOff() {
        super.addRequirements(Robot.coralArm);
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(ArmStates.HAND_OFF);
    }

    @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        double position = Robot.coralArm.getPositionRad();
        return UtilityFunctions.withinMargin(0.001, coralArmConstants.handOffSetPoint_rad, position);
    }
}
