package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class MoveDown extends Command {

    private Arm arm;
    
    public MoveDown(Arm arm) {
        this.arm = arm;
        super.addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.moveDown();
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
