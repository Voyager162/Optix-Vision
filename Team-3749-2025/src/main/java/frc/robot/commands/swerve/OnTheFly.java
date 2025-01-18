package frc.robot.commands.swerve;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.ToPos;

public class OnTheFly extends Command {

    private PathPlannerTrajectory trajectory = null;
    private final Timer timer = new Timer();
    private boolean pathGenerated = false;
    private boolean generationTimeout = false;

    private final PPHolonomicDriveController swerveController = new PPHolonomicDriveController(
        new PIDConstants(AutoConstants.kPDrive, 0, AutoConstants.kDDrive),
        new PIDConstants(AutoConstants.kPTurn, 0, AutoConstants.kDTurn)
    );

    public OnTheFly() {}

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        pathGenerated = false;
        generationTimeout = false;

        // Start path generation asynchronously to avoid blocking the main thread
        new Thread(() -> {
            try {
                System.out.println("Starting path generation...");
                PathPlannerPath path = ToPos.generateDynamicPath(
                    Robot.swerve.getPose(),
                    Robot.swerve.getPPSetpoint(),
                    Robot.swerve.getMaxDriveSpeed(),
                    SwerveConstants.DriveConstants.maxAccelerationMetersPerSecondSquared,
                    Robot.swerve.getMaxAngularSpeed(),
                    SwerveConstants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared,
                    new Translation2d(4.5, 4),
                    0.940816
                );

                if (path == null) {
                    System.out.println("Path generation failed: path is null.");
                    return;
                }

                trajectory = path.generateTrajectory(
                    Robot.swerve.getChassisSpeeds(),
                    Robot.swerve.getRotation2d(),
                    RobotConfig.fromGUISettings()
                );

                pathGenerated = true;
                System.out.println("Path generation completed successfully.");
            } catch (IOException | ParseException e) {
                System.out.println("Path generation failed: " + e.getMessage());
            }
        }).start();
    }

    @Override
    public void execute() {
        if (!pathGenerated) {
            if (timer.get() > 1.0 && !generationTimeout) {
                generationTimeout = true;
                System.out.println("Path generation taking longer than expected.");
            }
            return;
        }

        double currentTime = timer.get();
        PathPlannerTrajectoryState goalState = trajectory.sample(currentTime);

        ChassisSpeeds speeds = swerveController.calculateRobotRelativeSpeeds(Robot.swerve.getPose(), goalState);
        Robot.swerve.setModuleStates(SwerveConstants.DriveConstants.driveKinematics.toSwerveModuleStates(speeds));
        Robot.swerve.logSetpoints(goalState);

        if (isFinished()) {
            end(true);
            Robot.swerve.isOTF = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        System.out.println(interrupted ? "Path execution interrupted." : "Path execution completed.");
    }

    @Override
    public boolean isFinished() {
        if (trajectory == null) {
            return false;
        }

        boolean trajectoryComplete = timer.get() >= trajectory.getTotalTimeSeconds();

        if (trajectoryComplete) {
            double positionTolerance = 0.02; // Adjusted for better precision
            Translation2d currentPosition = Robot.swerve.getPose().getTranslation();
            Translation2d finalPosition = trajectory.getEndState().pose.getTranslation();
            double positionError = currentPosition.getDistance(finalPosition);

            return positionError <= positionTolerance;
        }

        return false;
    }
}
