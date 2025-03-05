package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;

/*
 * Handoff command for coral intake to chute
 * 
 * @author Dhyan Soni
 */
public class Climb extends Command {

    public Climb() {
        addRequirements(Robot.climbArm, Robot.elevator, Robot.coralArm);
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(ElevatorStates.L1);
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOW);
        Robot.climbArm.setState(ClimbArmConstants.ArmStates.CLIMB);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
