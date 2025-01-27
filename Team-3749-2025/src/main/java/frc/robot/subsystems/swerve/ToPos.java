package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ToPos {
    private static final ExecutorService executorService = Executors.newSingleThreadExecutor();

    public static void generateDynamicPathAsync(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration) {
        executorService.submit(() -> {
            System.out.println("Starting path generation...");
            PathPlannerPath path = generateDynamicPath(
                    initialPose, finalPose, maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);
            if (path != null) {
                System.out.println("Path generation completed successfully.");
                logWaypoints(path.getWaypoints());
            } else {
                System.out.println("Path generation failed.");
            }
        });
    }

    public static void shutdown() {
        executorService.shutdownNow();
        System.out.println("Path generation thread stopped.");
    }

    public static PathPlannerPath generateDynamicPath(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration) {
        // Create the list of waypoints
        List<Waypoint> waypoints = new ArrayList<>();

        // Add the initial waypoint
        Translation2d start = initialPose.getTranslation();
        waypoints.add(new Waypoint(start, start, start));

        // Add the final waypoint
        Translation2d end = finalPose.getTranslation();
        waypoints.add(new Waypoint(end, end, end));

        // Define path constraints
        PathConstraints constraints = new PathConstraints(
                maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration
        );

        try {
            // Generate the path with obstacle avoidance based on navgrid.json
            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null, // Optional initial state (not needed for dynamic paths)
                    new GoalEndState(0.0, finalPose.getRotation()) // Final goal state
            );

            path.preventFlipping = true; // Prevent unnecessary path flipping
            return path;
        } catch (Exception e) {
            System.out.println("PathPlanner Error: " + e.getMessage());
            return null;
        }
    }

    private static void logWaypoints(List<Waypoint> waypoints) {
        System.out.println("Generated Waypoints:");
        for (Waypoint waypoint : waypoints) {
            System.out.println("Waypoint Details: " + waypoint.toString());
    }   
    }
}
