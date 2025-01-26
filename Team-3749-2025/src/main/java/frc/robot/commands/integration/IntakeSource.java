package frc.robot.commands.integration;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.implementations.ScoringRoller;

public class IntakeSource extends Command {
    private final Roller scoringRoller;
    private final Elevator elevator;
    private final Chute chute;

    public IntakeSource(Roller scoringRoller, Elevator elevator, Chute chute) {
        this.scoringRoller = scoringRoller;
        this.elevator = elevator;
        this.chute = chute;
    }

    @Override
    public void initialize() {
        elevator.setState(ElevatorStates.SOURCE); 
        scoringRoller.setState(RollerConstants.RollerStates.MAINTAIN); 
    }

    @Override
    public void execute() {
        if (elevator.getState() == ElevatorStates.SOURCE && elevator.getIsStableState()) { 
            scoringRoller.setState(RollerConstants.RollerStates.RUN);
        }
        if (scoringRoller.hasPiece()){
            Robot.led.setLEDPattern(LEDPattern.GREEN);
        }
    }
 
    @Override
    public void end(boolean interrupted) {
        scoringRoller.setState(RollerConstants.RollerStates.STOP); 
    }

    @Override
    public boolean isFinished() {
        return chute.hasPiece();
    }
}
