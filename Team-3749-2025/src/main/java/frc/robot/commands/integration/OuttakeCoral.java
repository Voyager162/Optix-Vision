package frc.robot.commands.integration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArm;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.roller.Roller;

public class OuttakeCoral extends Command {

    public int delay_counter = 0;

    public OuttakeCoral() {
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }
    
    @Override
    public void initialize() {
        Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE);
        Robot.elevator.setState(ElevatorStates.STOW);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralRoller.setState(RollerConstants.RollerStates.STOP);
        System.out.println("DEBUG : outtake command end");
    }

    @Override
    public boolean isFinished() {
        if (delay_counter < 2) {
            delay_counter++;
            return false;
        } else {
            System.out.println("outtake coral: " + Robot.coralArm.hasPiece());
            return !Robot.coralArm.hasPiece();
        }
    }

}
