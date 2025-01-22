package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.roller.Roller;

public class IntakeFloor extends Command {
    private final CoralArm coralArm;
    private final Roller[] rollers;

    public IntakeFloor(CoralArm coralArm, Roller[] rollers) {
        this.coralArm = coralArm;
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralConstants.ArmStates.CORAL_PICKUP);
        for (Roller roller : rollers) {
            roller.setState(RollerConstants.RollerStates.MAINTAIN);
        }
    }

    @Override
    public void execute() {
        if (coralArm.getIsStableState()){
            rollers[1].setState(RollerConstants.RollerStates.RUN);
        }
    }

    @Override
    public void end(boolean interrupted) {
        coralArm.setState(CoralConstants.ArmStates.STOPPED);
        for (Roller roller : rollers) {
            roller.setState(RollerConstants.RollerStates.STOP);
        }
    }

    @Override
    public boolean isFinished() {
        return coralArm.hasPiece();
    }
}
