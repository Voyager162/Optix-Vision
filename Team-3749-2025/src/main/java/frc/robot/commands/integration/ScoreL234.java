package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;

//works for l2-4 change name
public class ScoreL234 extends Command {
    private final Chute chute;
    private final CoralArm coralArm;
    private final Elevator elevator;
    private final Roller[] rollers;
    private final ElevatorStates state;

    public ScoreL234(Chute chute, CoralArm coralArm, Elevator elevator, Roller[] rollers, ElevatorStates state) {
        this.chute = chute;
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.rollers = rollers;
        this.state = state;
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
        for (Roller roller : rollers) {
            roller.setState(RollerConstants.RollerStates.MAINTAIN);
        }
        if (chute.hasPiece() && coralArm.getState() == CoralConstants.ArmStates.HAND_OFF) {
            elevator.setState(ElevatorStates.STOW);
            System.out.println("elevator state stow");
        }
        System.out.println("init");
    }

    @Override
    public void execute() {
        System.out.println(chute.hasPiece());
        System.out.println(coralArm.getState());
        System.out.println(coralArm.getIsStableState());
        System.out.println(elevator.getState());
        System.out.println(elevator.getIsStableState());
        if (chute.hasPiece() && coralArm.getState() == CoralConstants.ArmStates.HAND_OFF && coralArm.getIsStableState()
                && elevator.getState() == ElevatorStates.STOW && elevator.getIsStableState()) {
            elevator.setState(state);
            System.out.println("elevator state set");
        }
        if (chute.hasPiece() && coralArm.getState() == CoralConstants.ArmStates.HAND_OFF
                && coralArm.getIsStableState() && elevator.getState() == state && elevator.getIsStableState()) {
            rollers[2].setState(RollerConstants.RollerStates.SCORE);
            System.out.println("score state set");
        }
        System.out.println("execute ran");
    }

    @Override
    public void end(boolean interrupted) {
        coralArm.setState(CoralConstants.ArmStates.STOPPED);
        elevator.setState(ElevatorStates.STOP);
        for (Roller roller : rollers) {
            roller.setState(RollerConstants.RollerStates.STOP);
        }
        System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
