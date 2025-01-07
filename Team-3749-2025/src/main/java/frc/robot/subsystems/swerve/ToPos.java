package frc.robot.subsystems.swerve;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
public class ToPos {
    
    public static PathPlannerPath generateDynamicPath(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration
    ) {
        // Create waypoints from the initial and final poses
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                initialPose,
                finalPose
        );
    
        // Define path constraints
        PathConstraints constraints = new PathConstraints(
                maxVelocity,
                maxAcceleration,
                maxAngularVelocity,
                maxAngularAcceleration
        );
    
        // Create the path with the specified constraints and goal end state
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // Ideal starting state, not needed for on-the-fly paths
                new GoalEndState(0.0, finalPose.getRotation()) // Goal end state with final heading
        );
    
        // Prevent the path from being flipped if coordinates are correct
        path.preventFlipping = true;
        return path;
    }
}
    
    
/** Main Goal
 *   
 *       we want to be able to move from anywhare on the field to anywhere else on the feild durring teleop using pathplanner and accurate trajectories to get to the
 *       setpoint without driver adjustment
 * 
 * 
 * Step one done by 1/7/2025
 *      Reaserch. What are we doing and how do we do it. 
 * Step two  
 *      whatever we do, high chances that there will be a initial state and end state. Make a method to geneorate trajecotry to end state from initial using a library. 
 * Step three 
 *       Already did this but get pos from bot and then use that as initial. 
 * Step four.
 *      Add constants as get end positions
 *      
**/