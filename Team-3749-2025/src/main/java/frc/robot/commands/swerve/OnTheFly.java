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

public class OnTheFly extends Command {

    private PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();
    private final PPHolonomicDriveController SwerveController = new PPHolonomicDriveController(
            new PIDConstants(AutoConstants.kPDrive, 0, AutoConstants.kDDrive),
            new PIDConstants(AutoConstants.kPTurn, 0, AutoConstants.kDTurn));
    private static double positionTolerance = 0.003; // meters
    private static double rotationTolerance = 2.0; // degrees

    public OnTheFly() {
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        ToPos toPos = new ToPos();
        PathPlannerPath path;

        path = toPos.generateDynamicPath(
                Robot.swerve.getPose(),
                Robot.swerve.getPPSetpoint().approachPoint,
                Robot.swerve.getPPSetpoint().setpoint,
                Robot.swerve.getMaxDriveSpeed(),
                SwerveConstants.DriveConstants.maxAccelerationMetersPerSecondSquared,
                Robot.swerve.getMaxAngularSpeed(),
                SwerveConstants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared);

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
            this.end(true);
            Robot.swerve.isOTF = false;

            // Optional: Add LEDs or any visual cue for completion here.
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
            return withinSetpointTolerance(trajectory.getEndState().pose);
        }

        return false;
    }

    /**
     * Ensures safe initialization of Rotation2d. Falls back to a default rotation
     * if invalid.
     *
     * @param rotation The rotation to validate.
     * @return A valid Rotation2d object.
     */
    private Rotation2d safeRotation(Rotation2d rotation) {
        if (Math.abs(rotation.getCos()) < 1e-6 && Math.abs(rotation.getSin()) < 1e-6) {
            System.out.println("Warning: Invalid Rotation2d detected. Falling back to neutral rotation.");
            return new Rotation2d(0); // Neutral rotation
        }
        return rotation;
    }

    /**
     * Checks if the robot's current pose is within tolerance of a given setpoint.
     *
     * @param setpoint The target pose to check against.
     * @return True if the robot is within the setpoint tolerance, false otherwise.
     */
    private boolean withinSetpointTolerance(Pose2d setpoint) {
        double xError = Math.abs(setpoint.relativeTo(Robot.swerve.getPose()).getX());
        double yError = Math.abs(setpoint.relativeTo(Robot.swerve.getPose()).getY());
        double thetaError = setpoint.relativeTo(Robot.swerve.getPose()).getRotation().getDegrees();

        return xError < positionTolerance && yError < positionTolerance && thetaError < rotationTolerance;
    }
}
