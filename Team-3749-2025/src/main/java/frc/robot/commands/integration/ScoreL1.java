package frc.robot.commands.integration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.utils.UtilityFunctions;

/*
 * ScoreL1 command for scoring coral on L1 using coral arm
 */
public class ScoreL1 extends Command {

    private double outtakeTimestamp = Double.MAX_VALUE;

    public ScoreL1() {
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(ElevatorConstants.ElevatorStates.STOW);
        Robot.coralArm.setState(CoralArmConstants.ArmStates.L1);
        Robot.coralRoller.setState(RollerConstants.RollerStates.MAINTAIN);
        Robot.scoringRoller.setState(RollerConstants.RollerStates.STOP);

    }

    @Override
    public void execute() {

        if (Robot.coralArm.getState() == CoralArmConstants.ArmStates.L1 && Robot.coralArm.getIsStableState()
                && outtakeTimestamp == Double.MAX_VALUE) {

            if ((Robot.swerve.getIsOTF() && !Robot.swerve.reachedSwerveSetpoint(Robot.swerve.getPPSetpoint().setpoint))
                    || (DriverStation.isAutonomous()
                            && !Robot.swerve.reachedSwerveSetpoint(Robot.swerve.getPositionSetpoint()))) {
                Robot.coralRoller.setState(RollerStates.STOP);
                return;
            }
            Robot.coralRoller.setState(RollerConstants.RollerStates.SCORE);
            outtakeTimestamp = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(CoralArmConstants.ArmStates.STOW);
        Robot.coralRoller.setState(RollerStates.STOP);
        outtakeTimestamp = Double.MAX_VALUE;
    }

    /**
     * Command finishes when coralRoller does not have coral and command is being
     * scheduled
     */
    @Override
    public boolean isFinished() {
        return Robot.coralRoller.getState() == RollerStates.SCORE && Robot.coralRoller.getIsStableState()
                && Timer.getFPGATimestamp() - outtakeTimestamp > 0.5 && this.isScheduled();
    }
}
