package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class MoveUp extends Command {

    private Arm arm;
    
    public MoveUp(Arm arm) {
        this.arm = arm;
        super.addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.moveUp();
    }

    @Override
    public void end(boolean interupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
