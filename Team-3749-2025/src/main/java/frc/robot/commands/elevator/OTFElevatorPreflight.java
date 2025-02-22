package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.utils.UtilityFunctions;

public class OTFElevatorPreflight extends Command {

    @Override
    public void initialize() {
        Robot.elevator.setState(ElevatorStates.STOW);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interupted) {
        
    }

    @Override
    public boolean isFinished() {
        return UtilityFunctions.withinMargin(ElevatorConstants.stateMarginOfError, Robot.elevator.getPositionMeters(), ElevatorConstants.StateHeights.stowHeight);
    }
}