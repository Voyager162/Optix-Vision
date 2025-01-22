package frc.robot.commands.integration;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;

public class IntakeSource extends Command {
    private final CoralArm coralArm;
    private final Roller[] rollers;
    private final Elevator elevator;
    private final Chute chute;

    public IntakeSource(CoralArm coralArm, Roller[] rollers, Elevator elevator, Chute chute) {
        this.coralArm = coralArm;
        this.rollers = rollers;
        this.elevator = elevator;
        this.chute = chute;
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralConstants.ArmStates.CORAL_PICKUP);
        elevator.setState(ElevatorStates.STOP); // stop or stow?
        for (Roller roller: rollers) {
            roller.setState(RollerConstants.RollerStates.MAINTAIN); 
        }
    }

    @Override
    public void execute() {
        if (coralArm.getState() == CoralConstants.ArmStates.CORAL_PICKUP && coralArm.getIsStableState() && elevator.getState() == ElevatorStates.STOP && elevator.getIsStableState()) {
            rollers[2].setState(RollerConstants.RollerStates.RUN);
            elevator.setState(ElevatorStates.SOURCE); 
        }
    }
 
    @Override
    public void end(boolean interrupted) {
        for (Roller roller: rollers) {
            roller.setState(RollerConstants.RollerStates.MAINTAIN); 
        }
    }

    @Override
    public boolean isFinished() {
            return chute.hasPiece() && elevator.getState() == ElevatorStates.SOURCE;
    }
}
