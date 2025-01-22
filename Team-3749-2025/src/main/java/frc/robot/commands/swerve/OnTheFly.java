package frc.robot.commands.swerve;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

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

    public OnTheFly() {
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        PathPlannerPath path = ToPos.generateDynamicPath(
                Robot.swerve.getPose(),
                Robot.swerve.getPPSetpoint(),
                Robot.swerve.getPPSetpoint(),
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
        if (trajectory == null) {
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

            double positionTolerance = 0.002; // meters
            double rotationTolerance = 2.0; // degrees
            // harik be like oooh i love doing position math that bricked our codebase
            // My Honest, Genuine, Unadulterated Built in Method Reaction
            double xError = trajectory.getEndState().pose.relativeTo(Robot.swerve.getPose()).getX();
            double yError = trajectory.getEndState().pose.relativeTo(Robot.swerve.getPose()).getY();
            double thetaError = trajectory.getEndState().pose.relativeTo(Robot.swerve.getPose()).getRotation()
                    .getDegrees();
            return xError < positionTolerance && yError < positionTolerance && thetaError < rotationTolerance;
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
}