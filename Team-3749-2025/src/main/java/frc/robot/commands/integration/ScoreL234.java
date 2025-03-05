package frc.robot.commands.integration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.utils.UtilityFunctions;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants.ArmStates;

/*
 * ScoreL234 command for scoring coral on L2, L3, L4
 */
@SuppressWarnings("unused")
public class ScoreL234 extends Command {
    private final ElevatorStates elevatorState;
    private boolean handoffComplete = false;
    private boolean pieceRecognized = false;
    private double scoreTimestamp = Double.MAX_VALUE;

    /**
     * 
     * @param elevatorState
     */
    public ScoreL234(ElevatorStates elevatorState) {
        this.elevatorState = elevatorState;
        // ensures other commands do not infere while this is active
        addRequirements(Robot.getAllSuperStructureSubsystems());
    }

    @Override
    public void initialize() {
        Robot.elevator.setState(elevatorState);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralArm.setState(ArmStates.STOW);

    }

    @Override
    public void execute() {
        if (Robot.scoringRoller.hasPiece()) {
            pieceRecognized = true;
        }

        // scores when elevator reaches desired state
        if (Robot.elevator.getState() == elevatorState && Robot.elevator.getIsStableState()
                && scoreTimestamp == Double.MAX_VALUE) {

            if ((Robot.swerve.getIsOTF() || DriverStation.isAutonomous()) && !reachedSwerveSetpoint()) {
                return;
            }

            Robot.scoringRoller.setState(RollerStates.SCORE);
            scoreTimestamp = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.coralArm.setState(ArmStates.STOW);
        Robot.elevator.setState(ElevatorStates.STOW);
        Robot.scoringRoller.setState(RollerStates.STOP);
        Robot.coralRoller.setState(RollerStates.STOP);
        pieceRecognized = false;
        scoreTimestamp = Double.MAX_VALUE;
    }

    /**
     * Command finishes when scoringRoller does not have coral and command is being
     * scheduled
     */
    @Override
    public boolean isFinished() {
        return !Robot.scoringRoller.hasPiece() && pieceRecognized && Timer.getFPGATimestamp() - scoreTimestamp > 0.3
                && this.isScheduled();
        // return false;
    }

    public boolean reachedSwerveSetpoint() {
        Pose2d setpoint = Robot.swerve.getPositionSetpoint();
        double xMargin = Math.sin(setpoint.getRotation().getRadians())
                * ToPosConstants.Setpoints.scoreWithinMarginMeters + 0.01;
        double yMargin = Math.cos(setpoint.getRotation().getRadians())
                * ToPosConstants.Setpoints.scoreWithinMarginMeters + 0.01;

        boolean withinPositionMargin = UtilityFunctions.withinMargin(
                new Pose2d(xMargin,
                        yMargin,
                        new Rotation2d(ToPosConstants.Setpoints.scoreWithinMarginRadians)),
                Robot.swerve.getPose(), setpoint);

        Pose2d velocities = new Pose2d(Robot.swerve.getChassisSpeeds().vxMetersPerSecond,
                Robot.swerve.getChassisSpeeds().vyMetersPerSecond,
                new Rotation2d(Robot.swerve.getChassisSpeeds().omegaRadiansPerSecond));

        boolean withinVelocityMargin = UtilityFunctions.withinMargin(
                new Pose2d(0.1, 0.1, new Rotation2d(Units.degreesToRadians(3))),
                Robot.swerve.getVelocitySetpoint(),
                velocities);

        return withinPositionMargin && withinVelocityMargin;

    }
}
