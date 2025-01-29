package frc.robot.commands.integration;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.algae.AlgaeConstants.ArmStates;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;

public class IntakeSource extends Command {

    public IntakeSource() {
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.coralArm.setState(CoralConstants.ArmStates.STOWED);
        Robot.elevator.setState(ElevatorStates.SOURCE); 
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN); 
    }

    @Override
    public void execute() {
        if (Robot.elevator.getState() == ElevatorStates.SOURCE && Robot.elevator.getIsStableState()) { 
            Robot.scoringRoller.setState(RollerConstants.RollerStates.RUN);
        }
    }
 
    @Override
    public void end(boolean interrupted) {
        Robot.scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN); 
        System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        System.out.println("intake source: " + Robot.scoringRoller.hasPiece());
        return Robot.scoringRoller.hasPiece();
    }
}
