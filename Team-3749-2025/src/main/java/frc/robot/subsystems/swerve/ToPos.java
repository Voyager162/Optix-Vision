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

    import com.pathplanner.lib.PathPlanner;
    import com.pathplanner.lib.PathPlannerTrajectory;
    import com.pathplanner.lib.PathConstraints;
    import com.pathplanner.lib.PathPoint;
    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.math.geometry.Translation2d;
    
    import java.util.List;
    
    public static PathPlannerTrajectory generateSwerveTrajectory(
            double initialX,
            double initialY,
            double initialHeading,
            double initialHolonomicRotation,
            double finalX,
            double finalY,
            double finalHeading,
            double finalHolonomicRotation,
            double maxVelocity,
            double maxAccel
    ) {
        // Create PathPoint objects for the start and end positions
        PathPoint startPoint = new PathPoint(
            new Translation2d(initialX, initialY),              // Start position
            Rotation2d.fromRadians(initialHeading),            // Start heading (direction of movement)
            Rotation2d.fromRadians(initialHolonomicRotation)   // Robot-facing direction at start
        );
    
        PathPoint endPoint = new PathPoint(
            new Translation2d(finalX, finalY),                 // End position
            Rotation2d.fromRadians(finalHeading),             // End heading (direction of movement)
            Rotation2d.fromRadians(finalHolonomicRotation)    // Robot-facing direction at end
        );
    
        // Generate the trajectory using PathPlanner's new API
        List<PathPoint> pathPoints = List.of(startPoint, endPoint); // Define the path points
        return PathPlanner.generatePath(
            new PathConstraints(maxVelocity, maxAccel), // Define path constraints
            pathPoints // Pass the list of points
        );
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