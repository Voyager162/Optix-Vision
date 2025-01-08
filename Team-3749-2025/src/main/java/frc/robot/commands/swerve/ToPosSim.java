package frc.robot.commands.swerve;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.ToPos;

public class ToPosSim {
FollowPathCommand followPathCommand = new FollowPathCommand(
    path,
    driveSubsystem::getPose,
    driveSubsystem::getChassisSpeeds,
    driveSubsystem::setChassisSpeeds,
    pathFollowingController,
    robotConfig,
    () -> false, // No path flipping
    driveSubsystem
);


//PPSwerveControllerCommand command = new PPSwerveControllerCommand(
//     path,
//     drivetrain::getPose, // Function to supply the robot's current pose
//     drivetrain.getKinematics(), // SwerveDriveKinematics
//     new PIDController(kP, kI, kD), // X PID controller
//     new PIDController(kP, kI, kD), // Y PID controller
//     new ProfiledPIDController(kP, kI, kD, constraints), // Theta PID controller
//     drivetrain::setModuleStates, // Method to set the desired swerve module states
//     drivetrain // The drivetrain subsystem
// );

/**
 * Im pretty sure this is where I actually run the trajectories. for that Ineed to figure out two things.
 * 1: is the swerve drive controller OBJECT(Hark- hopefully by end of ENM),
 * 2: is physically running with it (- John and Hark at meeting)
 * 3: is constants to endpoints to see it working in sim( this takes like an minute, but remember to do it correctly)
 * 4: is obvios, robot container button bindings (This is where hopefully it updates states with a push of a button so no need for a seperate sim file, 
 * but if we need one, we can see and it will just go here after this step )
 * 5: SUFFERING to debug it all (Both Hark an John)
 */

}