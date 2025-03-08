package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.arm.coral.CoralArmConstants;

/*
 * Handoff command for coral intake to chute
 * 
 * @author Dhyan Soni
 */
public class PrepareClimb extends Command {

    public PrepareClimb() {
        addRequirements(Robot.elevator, Robot.coralArm);
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(ElevatorStates.L2);
        Robot.coralArm.setState(CoralArmConstants.ArmStates.L1);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return Robot.elevator.getIsStableState() && Robot.coralArm.getIsStableState()&& this.isScheduled();
    }
}
