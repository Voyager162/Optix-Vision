package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class OuttakeRoller extends Command {
    private static RollerStates targetState = RollerStates.OUTTAKE;
    private Roller roller;

    public OuttakeRoller(Roller roller) {
        this.roller = roller;
        addRequirements(roller);

    }

    @Override
    public void initialize() {
        roller.setState(targetState);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        roller.setState(RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}