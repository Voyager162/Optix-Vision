package frc.robot.commands.integration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;

/*
 * KnockAlgae command for knocking algae off reef
 */
@SuppressWarnings("unused")
public class KnockAlgae extends Command {
    private final ElevatorStates elevatorState;
    private double startTimeStamp = Double.MAX_VALUE;
    private double minComandTime = 5;

    /**
     * 
     * @param elevatorState
     */
    public KnockAlgae(ElevatorStates elevatorState) {
        this.elevatorState = elevatorState;
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOW);
        Robot.scoringRoller.setIsAlgaeMode(true);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.INTAKE);
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.elevator.setState(elevatorState);
        startTimeStamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setIsAlgaeMode(false);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.STOP);
        startTimeStamp = Double.MAX_VALUE;
    }

    /**
     * Command finishes when elevator reaches desired state and command is being
     * scheduled
     */
    @Override
    public boolean isFinished() {
        return false;
        // return Robot.elevator.getState() == ElevatorStates.ALGAE_LOW && Robot.elevator.getIsStableState()
        //         && Timer.getFPGATimestamp() - startTimeStamp > minComandTime && this.isScheduled();
    }
}
