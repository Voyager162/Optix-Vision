package frc.robot.commands.integration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.sim.PhotoelectricSim;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;

//works for l2-4 change name
public class ScoreL234 extends Command {
    private final ElevatorStates state;
    private boolean handoffComplete = false;

    public ScoreL234(ElevatorStates state) {
        this.state = state;
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        if (Robot.scoringRoller.hasPiece()) {
            Robot.elevator.setState(state); 
            Robot.scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN); 
            Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
            System.out.println("DEBUG >>>>> ScoreL234: Initalize coralArm");
        } else if(Robot.coralArm.hasPiece()) {
            Robot.coralArm.setState(CoralConstants.ArmStates.HAND_OFF);
            Robot.elevator.setState(ElevatorStates.STOW);
            Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN); 
            System.out.println("DEBUG >>>>> ScoreL234: Initalize coralArm");
        } else {
            System.out.println("DEBUG >>>>> ScoreL234: Cancel");
            // this.cancel(); // HACK
        }
    }

    @Override
    public void execute() {
        if (Robot.coralArm.getState() == CoralConstants.ArmStates.HAND_OFF && Robot.coralArm.getIsStableState() && Robot.elevator.getState() == ElevatorStates.STOW) { // add working elevator isStableState later
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE); 
            handoffComplete = true;
        }
        if (handoffComplete && !Robot.coralArm.hasPiece() && Robot.scoringRoller.hasPiece()) { // should work once hasPiece logic is made
            Robot.elevator.setState(state);
        }
        if (Robot.elevator.getState() == state && Robot.elevator.getIsStableState()) {
            Robot.scoringRoller.setState(RollerConstants.RollerStates.SCORE); 
            // photoelectricSim.setSensing(this.scoreTimer, Robot.scoringRoller.hasPiece());
        }

    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralConstants.ArmStates.STOPPED);
        Robot.elevator.setState(ElevatorStates.STOP);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.STOP); 
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP); 
        System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        System.out.println("DEBUG >>>>> ScoreL234: " + Robot.coralArm.hasPiece());
        return Robot.coralArm.hasPiece();
        // return Robot.elevator.getState() == state && !Robot.scoringRoller.hasPiece(); // add working elevator isStableState later
    }
}
