package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.roller.Roller;

public class IntakeFloor extends Command {
    private final CoralArm coralArm;
    private final Roller coralRoller;

    public IntakeFloor(CoralArm coralArm, Roller coralRoller) {
        this.coralArm = coralArm;
        this.coralRoller = coralRoller;
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralConstants.ArmStates.CORAL_PICKUP);
    }

    @Override
    public void execute() {
        coralRoller.setState(RollerConstants.RollerStates.RUN);
    }

    @Override
    public void end(boolean interrupted) {
        coralArm.setState(CoralConstants.ArmStates.STOPPED);
        coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
    }

    @Override
    public boolean isFinished() {
        return coralArm.hasPiece();
    }
}
