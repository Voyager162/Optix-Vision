package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.buttons.ToPosTriggers;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;

public class OTFAuto extends Command {
    int otfIndex;

    boolean ranPath = false;

    public OTFAuto(int index) {
        addRequirements(Robot.getAllSuperStructureSubsystems());

        otfIndex = index;
    }

    @Override
    public void initialize() {
        Robot.swerve.startOnTheFly(otfIndex);
    }

    @Override
    public void execute() {
        if (!ranPath && ToPosTriggers.OTFWithinMargin()) {
            ranPath = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return ranPath && ToPosTriggers.OTFWithinMargin() && Robot.elevator.getCurrentCommand().isFinished();
    }
}
