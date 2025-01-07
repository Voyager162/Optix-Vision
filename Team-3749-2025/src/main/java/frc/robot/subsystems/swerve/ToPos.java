package frc.robot.subsystems.swerve;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

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
