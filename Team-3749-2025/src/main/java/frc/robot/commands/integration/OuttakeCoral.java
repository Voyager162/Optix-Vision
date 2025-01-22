package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.roller.Roller;

public class OuttakeCoral extends Command {
    private final CoralArm coralArm;
    private final Roller[] rollers;

    public OuttakeCoral(CoralArm coralArm, Roller[] rollers) {
        this.coralArm = coralArm;
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        for (Roller roller : rollers) {
            roller.setState(RollerConstants.RollerStates.MAINTAIN);
        }
    }

    @Override
    public void execute() {
        if (coralArm.hasPiece() && coralArm.getState() == CoralConstants.ArmStates.CORAL_PICKUP){
            rollers[1].setState(RollerConstants.RollerStates.SCORE);
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
        return !coralArm.hasPiece();
    }
}
