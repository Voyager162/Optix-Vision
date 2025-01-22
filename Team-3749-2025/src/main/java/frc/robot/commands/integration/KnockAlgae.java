package frc.robot.commands.integration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;

public class KnockAlgae extends Command{   
    private final CoralArm coralArm;
    private final Elevator elevator;
    private final Roller roller;
    private final SetElevatorState setElevatorState;
    private final ElevatorStates state;

    public KnockAlgae(CoralArm coralArm, Elevator elevator, Roller roller, ElevatorStates state) {
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.roller = roller;
        this.state = state;
        this.setElevatorState = new SetElevatorState(this.state);
    }

    @Override
    public void initialize() {
        coralArm.setState(CoralConstants.ArmStates.CORAL_PICKUP);
        elevator.setState(ElevatorStates.STOW);
        roller.setState(RollerConstants.RollerStates.MAINTAIN); 
    }

    @Override 
    public void execute() {
        CommandScheduler.getInstance().schedule(setElevatorState);

        //System.out.println(state);
        elevator.setState(state);
        if(elevator.getIsStableState()){
            roller.setState(RollerConstants.RollerStates.RUN);
        }
            
    }

    @Override 
    public void end(boolean interrupted) {
        coralArm.setState(CoralConstants.ArmStates.STOPPED);
        elevator.setState(ElevatorStates.STOP);
        roller.setState(RollerConstants.RollerStates.STOP); 
    }

    @Override
    public boolean isFinished() {
        return elevator.getState() == state;
    }
}
