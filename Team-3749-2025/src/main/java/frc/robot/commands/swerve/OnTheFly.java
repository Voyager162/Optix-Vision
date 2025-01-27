package frc.robot.commands.swerve;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.ToPos;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.subsystems.swerve.ToPosConstants.Setpoints.PPSetpoints;

public class OnTheFly extends Command {

    private PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private final PPHolonomicDriveController SwerveController = new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPDrive, 0, AutoConstants.kDDrive),
            new PIDConstants(AutoConstants.kPTurn, 0, AutoConstants.kDTurn));
    private static double positionTolerance = 0.003; // meters
    private static double rotationTolerance = 2.0; // degrees

    private PPSetpoints finalSetpoint;

    public OnTheFly(PPSetpoints setpoint) {
        finalSetpoint=setpoint
    }

    @Override
    public void initialize() {
        if (withinSetpointTolerance(ToPosConstants.Setpoints.reefTrig(Robot.swerve.getPPSetpoint(),
                ToPosConstants.Setpoints.TrigDirection.FORWARD), true)) {
            this.cancel();
            Robot.swerve.isOTF = false;
        }
        timer.reset();
        timer.start();
        ToPos ToPos = new ToPos();
        PathPlannerPath[] paths;

        paths = ToPos.generateDynamicPath(
                Robot.swerve.getPose(),
                finalSetpoint.approachPoint,
                finalSetpoint.setpoint,
                Robot.swerve.getMaxDriveSpeed(),
                SwerveConstants.DriveConstants.maxAccelerationMetersPerSecondSquared,
                Robot.swerve.getMaxAngularSpeed(),
                SwerveConstants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);

        for (PathPlannerPath path : paths) {
            if (path == null) {
                System.out.println("Error: Failed to generate path. Ending OnTheFly command.");
                Robot.swerve.isOTF = false;
                this.cancel();
                return;
            }

            try {
                trajectory = path.generateTrajectory(
                        Robot.swerve.getChassisSpeeds(),
                        safeRotation(Robot.swerve.getRotation2d()),
                        RobotConfig.fromGUISettings());
            } catch (IOException | ParseException e) {
                e.printStackTrace();
                Robot.swerve.isOTF = false;
                this.cancel();
            }
        }
    }

    @Override
    public void execute() {
        if (trajectory == null || !Robot.swerve.isOTF) {
            this.cancel();
            return;
        }

        double currentTime = timer.get();
        PathPlannerTrajectoryState goalState = trajectory.sample(currentTime);
        ChassisSpeeds speeds = SwerveController.calculateRobotRelativeSpeeds(Robot.swerve.getPose(), goalState);

        Robot.swerve.setModuleStates(SwerveConstants.DriveConstants.driveKinematics.toSwerveModuleStates(speeds));
        Robot.swerve.logSetpoints(goalState);

        if (isFinished()) {
            if (isAReefSetpoint && trajectory != null) {
                isAReefSetpoint = false;
                Robot.swerve.needsToCloseIn = true;
                this.end(true);
                this.initialize();
                return;
            }
            this.end(true);
            Robot.swerve.isOTF = false;

            // if we add LED's here now would be a good time to shine our bots lights to
            // signify you can move again
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        if (trajectory == null) {
            return true;
        }

        // Check if the timer has exceeded the trajectory duration
        boolean trajectoryComplete = timer.get() >= trajectory.getTotalTimeSeconds();

        if (trajectoryComplete) {
            return withinSetpointTolerance(trajectory.getEndState().pose, false);
        }

        return false;
    }

    /**
     * Ensures safe initialization of Rotation2d. Falls back to a default rotation
     * if invalid.
     */
    private Rotation2d safeRotation(Rotation2d rotation) {
        if (Math.abs(rotation.getCos()) < 1e-6 && Math.abs(rotation.getSin()) < 1e-6) {
            System.out.println("Warning: Invalid Rotation2d detected. Falling back to neutral rotation.");
            return new Rotation2d(0); // Neutral rotation
        }
        return rotation;
    }

    private boolean withinSetpointTolerance(Pose2d setpoint, boolean useAlternateTolerance) {
        // harik be like oooh i love doing position math that bricked our codebase
        // My Honest, Genuine, Unadulterated Built in Method Reaction
        double alternatePositionTolerance = positionTolerance;
        if (useAlternateTolerance) {
            alternatePositionTolerance = 0.17;
        }
        double xError = Math.abs(setpoint.relativeTo(Robot.swerve.getPose()).getX());
        double yError = Math.abs(setpoint.relativeTo(Robot.swerve.getPose()).getY());
        double thetaError = setpoint.relativeTo(Robot.swerve.getPose()).getRotation()
                .getDegrees();
        // System.out.println("x "+xError+" y "+yError+" theta " + thetaError)
        return xError < alternatePositionTolerance && yError < alternatePositionTolerance
                && thetaError < rotationTolerance;
    }
}