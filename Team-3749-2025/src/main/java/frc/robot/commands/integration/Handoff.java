package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;
import frc.robot.commands.elevator.SetElevatorState;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Handoff extends Command {
    private final Chute chute;    
    private final CoralArm coralArm;
    private final Elevator elevator;
    private final Roller[] rollers;
    private final SetElevatorState setElevatorState;

    public Handoff(Chute chute, CoralArm coralArm, Elevator elevator, Roller[] rollers) {
        this.chute = chute;
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.rollers = rollers;
        this.setElevatorState = new SetElevatorState(ElevatorStates.STOW);
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralConstants.ArmStates.CORAL_PICKUP);
        for (Roller roller : rollers) {
            roller.setState(RollerConstants.RollerStates.MAINTAIN);
        }
    }

    @Override
    public void execute() {
        if (coralArm.getIsStableState()){
            rollers[1].setState(RollerConstants.RollerStates.RUN);
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
        return coralArm.hasPiece();
    }
}
