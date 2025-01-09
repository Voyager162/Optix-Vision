package frc.robot.subsystems.swerve;

import java.util.List;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ToPos {

    public static PathPlannerPath generateDynamicPath(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration,
            Translation2d obstacleCenter,
            double obstacleRadius
    ) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(initialPose, finalPose);

        if (intersectsObstacle(initialPose.getTranslation(), finalPose.getTranslation(), obstacleCenter, obstacleRadius)) {
            Translation2d detour = calculateDetour(initialPose.getTranslation(), finalPose.getTranslation(), obstacleCenter, obstacleRadius);
            Translation2d tangent = calculateTangent(initialPose.getTranslation(), detour);
            double controlPointDistance = 1.0;

            Translation2d prevControl = detour.minus(tangent.times(controlPointDistance));
            Translation2d nextControl = detour.plus(tangent.times(controlPointDistance));

            waypoints.add(1, new Waypoint(prevControl, detour, nextControl));
        }

        PathConstraints constraints = new PathConstraints(
                maxVelocity,
                maxAcceleration,
                maxAngularVelocity,
                maxAngularAcceleration
        );

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, finalPose.getRotation())
        );

        path.preventFlipping = true;
        return path;
    }

    private static boolean intersectsObstacle(Translation2d start, Translation2d end, Translation2d center, double radius) {
        double distToLine = Math.abs((end.getY() - start.getY()) * center.getX() -
                                     (end.getX() - start.getX()) * center.getY() +
                                     end.getX() * start.getY() -
                                     end.getY() * start.getX()) /
                            start.getDistance(end);
        return distToLine < radius;
    }

    private static Translation2d calculateDetour(Translation2d start, Translation2d end, Translation2d center, double radius) {
        Translation2d direction = new Translation2d(end.getX() - start.getX(), end.getY() - start.getY()).div(start.getDistance(end));
        Translation2d perpendicular = new Translation2d(-direction.getY(), direction.getX()).times(radius + 0.5);
        return center.plus(perpendicular);
    }

    private static Translation2d calculateTangent(Translation2d start, Translation2d end) {
        Translation2d vector = new Translation2d(end.getX() - start.getX(), end.getY() - start.getY());
        return vector.div(vector.getNorm());
    }
}
