package frc.robot.commands.arm.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmConstants.climbArmConstants;
import frc.robot.subsystems.arm.ArmConstants.climbArmConstants.ArmStates;
import frc.robot.utils.UtilityFunctions;

public class ClimbArmPrepareForClimb extends Command {

    public ClimbArmPrepareForClimb() {
        super.addRequirements(Robot.climbArm);
    }

    @Override
    public void initialize() {
        Robot.climbArm.setState(ArmStates.PREPARE_FOR_CLIMB);
    }

    @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        double position = Robot.climbArm.getPositionRad();
        return UtilityFunctions.withinMargin(0.001, climbArmConstants.climbSetPoint_rad, position);
    }
}
