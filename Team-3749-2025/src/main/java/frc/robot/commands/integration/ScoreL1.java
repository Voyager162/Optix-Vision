package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class ScoreL1 extends Command {
    private final CoralArm coralArm;
    private final Roller[] rollers;

    public ScoreL1 (CoralArm coralArm, Roller[] rollers) {
        this.coralArm = coralArm;
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralConstants.ArmStates.STOWED);
        rollers[1].setState(RollerConstants.RollerStates.MAINTAIN); 
    }

    @Override
    public void execute() {
        if (coralArm.getState() == CoralConstants.ArmStates.STOWED && coralArm.getIsStableState()) {
            coralArm.setState(CoralConstants.ArmStates.CORAL_PICKUP);
            rollers[1].setState(RollerConstants.RollerStates.RUN);
        }
        if (coralArm.getState() == CoralConstants.ArmStates.CORAL_PICKUP && coralArm.getIsStableState() && coralArm.hasPiece()) {
            rollers[1].setState(RollerConstants.RollerStates.MAINTAIN);
            coralArm.setState(CoralConstants.ArmStates.L1);
        }
    }

    @Override 
    public void end(boolean interrupted) {
        coralArm.setState(CoralConstants.ArmStates.STOWED);
        rollers[1].setState(RollerStates.STOP);
    }

    @Override 
    public boolean isFinished() {
        return coralArm.getState() == CoralConstants.ArmStates.L1;
    }
}
