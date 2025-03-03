package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class StopRoller extends Command {
    private static RollerStates targetState = RollerStates.STOP;

    public StopRoller() {

    }

    @Override
    public void initialize() {
        Robot.coralRoller.setState(targetState);
        Robot.scoringRoller.setState(targetState);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralRoller.setState(RollerStates.STOP);
        Robot.scoringRoller.setState(RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}