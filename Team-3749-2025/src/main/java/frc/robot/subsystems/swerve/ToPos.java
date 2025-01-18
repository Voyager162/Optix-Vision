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
        List<Waypoint> waypoints = new ArrayList<>();

        // Add initial pose
        waypoints.add(new Waypoint(initialPose.getTranslation(), initialPose.getTranslation(), initialPose.getTranslation()));

        // Calculate detours for obstacle avoidance
        List<Translation2d> detours = calculateOptimizedDetours(
                initialPose.getTranslation(), finalPose.getTranslation(), obstacleCenter, obstacleRadius
        );

        if (detours != null && !detours.isEmpty()) {
            for (Translation2d detour : detours) {
                waypoints.add(new Waypoint(detour, detour, detour));
                System.out.println("Added detour waypoint: " + detour);
            }
        } else {
            System.out.println("No detours necessary. Using direct path.");
        }

        // Add final pose
        waypoints.add(new Waypoint(finalPose.getTranslation(), finalPose.getTranslation(), finalPose.getTranslation()));

        // Validate waypoints
        if (waypoints.size() < 2) {
            System.out.println("Invalid path: Not enough waypoints.");
            return null;
        }

        // Generate and return the trajectory
        try {
            PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);
            PathPlannerPath path = new PathPlannerPath(
                    waypoints, constraints, null, new GoalEndState(0.0, finalPose.getRotation()));

            path.preventFlipping = true; // Prevents path flipping issues
            System.out.println("Path generation complete. Total waypoints: " + waypoints.size());
            return path;
        } catch (Exception e) {
            System.out.println("Error during trajectory generation: " + e.getMessage());
            return null;
        }
    }

    private static List<Translation2d> calculateOptimizedDetours(
            Translation2d start, Translation2d end, Translation2d center, double radius) {

        List<Translation2d> detours = new ArrayList<>();

        // Vector-based approach to detect obstacle and calculate detours
        Translation2d direction = end.minus(start).div(start.getDistance(end));
        Translation2d obstacleVector = center.minus(start);
        double projection = (obstacleVector.getX() * direction.getX() + obstacleVector.getY() * direction.getY());
        Translation2d closestPoint = start.plus(direction.times(projection));

        double distanceToCenter = closestPoint.getDistance(center);

        if (distanceToCenter < radius) {
            System.out.println("Obstacle detected. Calculating detour...");

            // Calculate tangent points around the obstacle
            double offsetDistance = Math.sqrt(radius * radius - distanceToCenter * distanceToCenter);
            Translation2d perpendicular = new Translation2d(-direction.getY(), direction.getX()).times(offsetDistance);

            Translation2d detour1 = closestPoint.plus(perpendicular);
            Translation2d detour2 = closestPoint.minus(perpendicular);

            // Select the shortest detour based on total path length
            double detour1Length = detour1.getDistance(start) + detour1.getDistance(end);
            double detour2Length = detour2.getDistance(start) + detour2.getDistance(end);

            if (detour1Length < detour2Length) {
                detours.add(detour1);
            } else {
                detours.add(detour2);
            }

            System.out.println("Detour points calculated: " + detours);
        } else {
            System.out.println("No obstacle interference detected.");
        }

        return detours;
    }
}
