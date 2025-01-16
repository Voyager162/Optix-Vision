package frc.robot.commands.arm.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.algaeArmConstants;
import frc.robot.subsystems.arm.ArmConstants.algaeArmConstants.ArmStates;
import frc.robot.utils.UtilityFunctions;

public class AlgaePickup extends Command {

    public AlgaePickup() {
        super.addRequirements(Robot.algaeArm);
    }

    @Override
    public void initialize() {
        Robot.algaeArm.setState(ArmStates.ALGAE_PICKUP);
    }

        @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        double position = Robot.algaeArm.getPositionRad();
        return UtilityFunctions.withinMargin(0.001, algaeArmConstants.algaePickUpSetPoint_rad, position);
    }
}
