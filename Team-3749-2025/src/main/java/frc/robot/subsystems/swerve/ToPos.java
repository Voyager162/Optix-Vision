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
        waypoints.add(new Waypoint(initialPose.getTranslation(), initialPose.getTranslation(), initialPose.getTranslation()));

        // Check if the direct path intersects the obstacle
        if (intersectsObstacle(initialPose.getTranslation(), finalPose.getTranslation(), obstacleCenter, obstacleRadius)) {
            System.out.println("Obstacle detected! Adding detour waypoint...");

            // Calculate detours
            List<Translation2d> detours = calculateOptimizedDetours(
                initialPose.getTranslation(), finalPose.getTranslation(), obstacleCenter, obstacleRadius
            );

            if (detours != null && !detours.isEmpty()) {
                for (Translation2d detour : detours) {
                    waypoints.add(new Waypoint(detour, detour, detour));
                    System.out.println("Added detour waypoint: " + detour);
                }
            } else {
                System.out.println("Failed to calculate detour. Path generation aborted.");
                return null;
            }
        } else {
            System.out.println("No obstacle detected. Using direct path.");
        }

        // Add final waypoint
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

            path.preventFlipping = true;
            System.out.println("Path generation complete. Total waypoints: " + waypoints.size());
            return path;
        } catch (Exception e) {
            System.out.println("Error during trajectory generation: " + e.getMessage());
            return null;
        }
    }

    private static boolean intersectsObstacle(Translation2d start, Translation2d end, Translation2d center, double radius) {
        Translation2d direction = end.minus(start).div(start.getDistance(end));
        Translation2d toObstacle = center.minus(start);

        double projection = toObstacle.getX() * direction.getX() + toObstacle.getY() * direction.getY();
        Translation2d closestPoint = start.plus(direction.times(projection));

        double distanceToObstacle = closestPoint.getDistance(center);
        boolean intersects = distanceToObstacle < radius && projection > 0 && projection < start.getDistance(end);

        if (intersects) {
            System.out.println("Intersection detected! Closest point: " + closestPoint + ", Distance to obstacle: " + distanceToObstacle);
        }

        return intersects;
    }

    private static List<Translation2d> calculateOptimizedDetours(Translation2d start, Translation2d end, Translation2d center, double radius) {
        // Generate two possible detours around the obstacle
        Translation2d direction = end.minus(start).div(start.getDistance(end));
        Translation2d perpendicular = new Translation2d(-direction.getY(), direction.getX());

        Translation2d detour1 = center.plus(perpendicular.times(radius + 0.5));
        Translation2d detour2 = center.minus(perpendicular.times(radius + 0.5));

        // Calculate costs (distance + deviation angle)
        double cost1 = detour1.getDistance(start) + detour1.getDistance(end);
        double cost2 = detour2.getDistance(start) + detour2.getDistance(end);

        // Choose the lower-cost detour
        List<Translation2d> selectedDetour = new ArrayList<>();
        if (cost1 < cost2) {
            selectedDetour.add(detour1);
        } else {
            selectedDetour.add(detour2);
        }

        // Add intermediate waypoints for smooth transitions
        Translation2d midPoint = center.plus(direction.times(radius + 1.0));
        selectedDetour.add(midPoint);

        return selectedDetour;
    }
}
