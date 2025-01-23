package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.roller.Roller;

/*
 * Handoff command for coral intake to chute
 * 
 * @author Dhyan Soni
 */

public class Handoff extends Command {
    private final Chute chute;
    private final CoralArm coralArm;
    private final Elevator elevator;
    private final Roller[] rollers;

    public Handoff(Chute chute, CoralArm coralArm, Elevator elevator, Roller[] rollers) {
        this.chute = chute;
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        System.out.println("init");
        coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
        for (Roller roller : rollers) {
            roller.setState(RollerConstants.RollerStates.MAINTAIN);
        }
    }

    @Override
    public void execute() {
        if (coralArm.getIsStableState() && elevator.getIsStableState()){
            System.out.println("stow");
            elevator.setState(ElevatorConstants.ElevatorStates.STOW);
        }
        if (coralArm.getIsStableState() && elevator.getIsStableState() && elevator.getState() == ElevatorStates.STOW && coralArm.getState() == CoralConstants.ArmStates.HAND_OFF){
            System.out.println("roller");
            rollers[1].setState(RollerConstants.RollerStates.RUN);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("end");
        coralArm.setState(CoralConstants.ArmStates.STOPPED);
        rollers[1].setState(RollerConstants.RollerStates.STOP);
    }

    @Override
    public boolean isFinished() {
        System.out.println("finished");
        return chute.hasPiece();
    }
}
