package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ToPos {

    public static class Field {
        private final List<Translation2d> vertices;
        private final Translation2d obstacleCenter;
        private final double obstacleRadius;

        public Field(Translation2d obstacleCenter, double obstacleRadius) {
            // Define the vertices of the octagonal field
            vertices = List.of(
                new Translation2d(0,0 ),   
                new Translation2d(0, 8),  
                new Translation2d(8, 8), 
                new Translation2d(8, 0), 
                new Translation2d(0, 0)
                
            );
            this.obstacleCenter = obstacleCenter;
            this.obstacleRadius = obstacleRadius;
        }

        public boolean isInsideField(Translation2d point) {
            // Check if the point is within the octagonal field using ray-casting algorithm
            int intersections = 0;
            for (int i = 0; i < vertices.size(); i++) {
                Translation2d v1 = vertices.get(i);
                Translation2d v2 = vertices.get((i + 1) % vertices.size());

                if ((point.getY() > Math.min(v1.getY(), v2.getY())) &&
                    (point.getY() <= Math.max(v1.getY(), v2.getY())) &&
                    (point.getX() <= Math.max(v1.getX(), v2.getX())) &&
                    v1.getY() != v2.getY()) {

                    double xinters = (point.getY() - v1.getY()) * (v2.getX() - v1.getX()) / (v2.getY() - v1.getY()) + v1.getX();
                    if (v1.getX() == v2.getX() || point.getX() <= xinters) {
                        intersections++;
                    }
                }
            }
            return intersections % 2 != 0;
        }

        public boolean intersectsObstacle(Translation2d start, Translation2d end) {
            double distToLine = Math.abs((end.getY() - start.getY()) * obstacleCenter.getX() -
                                         (end.getX() - start.getX()) * obstacleCenter.getY() +
                                         end.getX() * start.getY() -
                                         end.getY() * start.getX()) /
                                start.getDistance(end);
            return distToLine < obstacleRadius;
        }
    }

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
        Field field = new Field(obstacleCenter, obstacleRadius);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(initialPose, finalPose);

        // Check if the path intersects the field boundary or obstacle
        if (!field.isInsideField(initialPose.getTranslation()) || 
            !field.isInsideField(finalPose.getTranslation()) || 
            field.intersectsObstacle(initialPose.getTranslation(), finalPose.getTranslation())) {
            Translation2d detour = calculateDetour(initialPose.getTranslation(), finalPose.getTranslation(), obstacleCenter, obstacleRadius);
            if (detour != null) {
                waypoints.add(1, new Waypoint(detour, detour, detour));
            }
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

    private static Translation2d calculateDetour(Translation2d start, Translation2d end, Translation2d center, double radius) {
        Translation2d direction = new Translation2d(end.getX() - start.getX(), end.getY() - start.getY()).div(start.getDistance(end));
        Translation2d perpendicular = new Translation2d(-direction.getY(), direction.getX()).times(radius + 0.5);
        return center.plus(perpendicular);
    }
}
