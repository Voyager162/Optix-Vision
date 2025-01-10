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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.ToPos;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;

public class OnTheFly extends Command {

    PathPlannerTrajectory trajectory;
    Timer timer = new Timer();
    PPHolonomicDriveController SwerveController = new PPHolonomicDriveController(
        new PIDConstants(AutoConstants.kPDrive, 0, AutoConstants.kDDrive), 
        new PIDConstants(AutoConstants.kPTurn,0,AutoConstants.kDTurn));

    public OnTheFly() {
        super.addRequirements(Robot.swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        PathPlannerPath path = ToPos.generateDynamicPath(Robot.swerve.getPose(),
            new Pose2d(1, 2, new Rotation2d(Math.toRadians(90))),
            Robot.swerve.getMaxDriveSpeed(), SwerveConstants.DriveConstants.maxAccelerationMetersPerSecondSquared,
            Robot.swerve.getMaxAngularSpeed(), SwerveConstants.DriveConstants.maxAngularAccelerationRadiansPerSecondSquared,new Translation2d(4.5,4),2);
        try {
            trajectory = path.generateTrajectory(Robot.swerve.getChassisSpeeds(), Robot.swerve.getRotation2d(), 
            RobotConfig.fromGUISettings());
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        PathPlannerTrajectoryState goalState = trajectory.sample(currentTime);
        ChassisSpeeds speeds = SwerveController.calculateRobotRelativeSpeeds(Robot.swerve.getPose(), goalState);
        Robot.swerve.setModuleStates(DriveConstants.driveKinematics.toSwerveModuleStates(speeds));
        Robot.swerve.logSetpoints(goalState);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
