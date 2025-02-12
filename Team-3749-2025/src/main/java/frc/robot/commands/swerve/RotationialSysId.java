package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

public class RotationialSysId extends Command {
    Command tuner;
    Swerve swerve;
    private final Timer timer = new Timer();
    private boolean startSysId = false;

    public RotationialSysId(Command tuner, Swerve swerve) {
        super.addRequirements(Robot.swerve);
        this.tuner = tuner;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // if further state change are needed over time, put them here with conditionals
    @Override
    public void execute() {
        swerve.setRotation();

        if (swerve.getRotated() && timer.hasElapsed(1)) {
            startSysId = true;
        }

        if (startSysId)
            tuner.schedule();

    }

    @Override
    public void end(boolean interupted) {
        startSysId = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
