package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;

public class ScoreL2 extends Command {
    private final Chute chute;    
    private final CoralArm coralArm;
    private final Elevator elevator;
    private final Roller[] rollers;
    private final SetElevatorState setElevatorState;

    public ScoreL2(Chute chute, CoralArm coralArm, Elevator elevator, Roller[] rollers) {
        this.chute = chute;
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.rollers = rollers;
        this.setElevatorState = new SetElevatorState(ElevatorStates.L2);
    }

    @Override
    public void initialize() {
        if (chute.hasPiece()) {
            coralArm.setState(CoralConstants.ArmStates.CORAL_PICKUP);
            elevator.setState(ElevatorStates.STOP); // not sure what stow is, maybe should be maintain?
            for (Roller roller: rollers) {
                roller.setState(RollerConstants.RollerStates.MAINTAIN); 
            }
        } else {
            coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
            elevator.setState(ElevatorStates.STOW);
            for (Roller roller: rollers) {
                roller.setState(RollerConstants.RollerStates.MAINTAIN); 
            }
        }
    }

    @Override 
    public void execute() {
        if (chute.hasPiece() && coralArm.getState() == CoralConstants.ArmStates.CORAL_PICKUP && coralArm.getIsStableState()) {
            CommandScheduler.getInstance().schedule(setElevatorState);
        }
        if (chute.hasPiece() && coralArm.getState() == CoralConstants.ArmStates.CORAL_PICKUP && coralArm.getIsStableState() && elevator.getState() == ElevatorStates.L2 && elevator.getIsStableState()) {
            rollers[2].setState(RollerConstants.RollerStates.SCORE); 
        }
    }

    @Override 
    public void end(boolean interrupted) {
        coralArm.setState(CoralConstants.ArmStates.STOPPED);
        elevator.setState(ElevatorStates.STOP);
        for (Roller roller: rollers) {
            roller.setState(RollerConstants.RollerStates.STOP); 
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.getState() == ElevatorStates.L2 && !chute.hasPiece();
    }

}
