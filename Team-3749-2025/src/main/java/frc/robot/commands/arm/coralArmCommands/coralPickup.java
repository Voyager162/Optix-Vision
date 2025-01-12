package frc.robot.commands.arm.coralArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.coralArmConstants;
import frc.robot.subsystems.arm.ArmConstants.coralArmConstants.ArmStates;
import frc.robot.utils.UtilityFunctions;

public class CoralPickup extends Command {

    public CoralPickup() {
        super.addRequirements(Robot.coralArm);
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(ArmStates.CORAL_PICKUP);
    }

        @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        double position = Robot.coralArm.getPositionRad();
        return UtilityFunctions.withinMargin(0.001, coralArmConstants.coralPickUpSetPoint_Rad, position);
    }
}
