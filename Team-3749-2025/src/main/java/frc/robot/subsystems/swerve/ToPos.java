package frc.robot.subsystems.swerve;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
public class ToPos {

    public ToPos()
    {

    }

    private final GoalEndState EndGoalTest = new GoalEndState(0.0,Rotation2d.fromDegrees(90));
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses();
    PathConstraints constaints = new PathConstraints(null, null, null, null);

    public void generatePathToPose(Pose2d pose) {
        PathPlannerPath path = new PathPlannerPath(waypoints,constaints,null,EndGoalTest);
        path.preventFlipping = true;
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