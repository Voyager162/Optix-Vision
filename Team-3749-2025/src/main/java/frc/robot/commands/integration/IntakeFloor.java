package frc.robot.commands.integration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

/*
 * IntakeFloor command for intaking coral from floor
 */
public class IntakeFloor extends Command {

    private double timeHasPiece = 0;

    public IntakeFloor() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.CORAL_PICKUP);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.coralRoller.setState(RollerConstants.RollerStates.INTAKE);
        Robot.scoringRoller.setState(RollerStates.STOP);
    }

    @Override
    public void execute() {
        if (Robot.coralRoller.hasPiece() && timeHasPiece == 0) {
            timeHasPiece = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOWED);
        Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
    }

    /**
     * Command finishes when coralRoller has coral and command is being scheduled
     */
    @Override
    public boolean isFinished() {
        return Robot.coralRoller.hasPiece() && this.isScheduled() && Timer.getFPGATimestamp() - timeHasPiece >= 1;
    }
}
