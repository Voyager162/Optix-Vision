package frc.robot.commands.integration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

/**
 * CoralIntakeSource command for intaking coral from source
 */
public class CoralIntakeSource extends Command {

    private double hasPieceTimeStamp = Double.MAX_VALUE;

    public CoralIntakeSource() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.CORAL_STATION);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.INTAKE);
        System.out.println("Source intake init");
        Robot.coralRoller.setHasPiece(false);

    }

    @Override
    public void execute() {
        System.out.println("Source intake ex");

        if (Robot.coralRoller.hasPiece() && hasPieceTimeStamp == Double.MAX_VALUE
                && hasPieceTimeStamp == Double.MAX_VALUE) {
            hasPieceTimeStamp = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - hasPieceTimeStamp > 0.4) {

            Robot.coralArm.setState(CoralArmConstants.ArmStates.STOW);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralRoller.setState(RollerStates.MAINTAIN);
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOW);

        hasPieceTimeStamp = Double.MAX_VALUE;

    }

    /**
     * Command finishes when coralRoller has coral and command is being scheduled
     */
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - hasPieceTimeStamp > 0.45 && this.isScheduled();
    }
}
