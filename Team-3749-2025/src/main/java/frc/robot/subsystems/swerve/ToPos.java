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

    private static final Translation2d HEXAGON_CENTER = new Translation2d(4.5, 4);
    private static final double HEXAGON_RADIUS = 1.0; // Radius of the hexagon
    private static final double SAFETY_MARGIN = 0.3; // Increased margin for better clearance

    // Closed-loop hexagon obstacle
    private static final List<Translation2d> HEXAGON_VERTICES = List.of(
        new Translation2d(4.5 + HEXAGON_RADIUS + SAFETY_MARGIN, 4),                     // Right
        new Translation2d(4.5 + 0.5 * HEXAGON_RADIUS + SAFETY_MARGIN, 4 + Math.sqrt(3) / 2 * HEXAGON_RADIUS + SAFETY_MARGIN),  // Top-right
        new Translation2d(4.5 - 0.5 * HEXAGON_RADIUS - SAFETY_MARGIN, 4 + Math.sqrt(3) / 2 * HEXAGON_RADIUS + SAFETY_MARGIN),  // Top-left
        new Translation2d(4.5 - HEXAGON_RADIUS - SAFETY_MARGIN, 4),                     // Left
        new Translation2d(4.5 - 0.5 * HEXAGON_RADIUS - SAFETY_MARGIN, 4 - Math.sqrt(3) / 2 * HEXAGON_RADIUS - SAFETY_MARGIN),  // Bottom-left
        new Translation2d(4.5 + 0.5 * HEXAGON_RADIUS + SAFETY_MARGIN, 4 - Math.sqrt(3) / 2 * HEXAGON_RADIUS - SAFETY_MARGIN),  // Bottom-right
        new Translation2d(4.5 + HEXAGON_RADIUS + SAFETY_MARGIN, 4)                      // Close loop
    );

    public static PathPlannerPath generateDynamicPath(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration
    ) {
        // Waypoints for PathPlanner
        List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(
            initialPose.getTranslation(),
            initialPose.getTranslation(),
            initialPose.getTranslation()
        ));

        // Add intermediate detour points if necessary
        List<Translation2d> detours = calculateDetours(initialPose.getTranslation(), finalPose.getTranslation());
        for (Translation2d detour : detours) {
            waypoints.add(new Waypoint(detour, detour, detour));
        }

        // Add final pose
        waypoints.add(new Waypoint(
            finalPose.getTranslation(),
            finalPose.getTranslation(),
            finalPose.getTranslation()
        ));

        // Ensure valid waypoints for PathPlanner
        if (waypoints.size() < 2) {
            System.out.println("Error: Not enough waypoints for path generation.");
            return null;
        }

        // Generate the path using PathPlanner
        PathConstraints constraints = new PathConstraints(
            maxVelocity,
            maxAcceleration,
            maxAngularVelocity,
            maxAngularAcceleration
        );

        try {
            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, finalPose.getRotation())
            );
            path.preventFlipping = true;
            System.out.println("Path generation completed successfully.");
            return path;
        } catch (Exception e) {
            System.out.println("PathPlanner Error: " + e.getMessage());
            return null;
        }
    }

    private static List<Translation2d> calculateDetours(Translation2d start, Translation2d end) {
        List<Translation2d> detours = new ArrayList<>();
        Translation2d currentStart = start; // Dynamically updated start point
        boolean obstacleDetected = false;
    
        while (true) {
            boolean intersectionFound = false;
    
            for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
                Translation2d vertex1 = HEXAGON_VERTICES.get(i);
                Translation2d vertex2 = HEXAGON_VERTICES.get(i + 1);
    
                // Check if the line intersects the obstacle edge
                if (linesIntersect(currentStart, end, vertex1, vertex2)) {
                    intersectionFound = true;
                    obstacleDetected = true;
                    System.out.println("Obstacle detected! Intersecting edge: " + vertex1 + " to " + vertex2);
    
                    // Calculate detour point
                    Translation2d detour = calculateBestDetour(currentStart, end, vertex1, vertex2);
    
                    if (detour != null) {
                        detours.add(detour);
                        System.out.println("Added detour waypoint: " + detour);
    
                        // Update the current start for further calculations
                        currentStart = detour;
                        break; // Re-evaluate from the new starting point
                    } else {
                        System.out.println("Failed to calculate a valid detour.");
                        return detours; // Return early if no valid detour can be made
                    }
                }
            }
    
            // If no intersections are found, the path is clear
            if (!intersectionFound) {
                break;
            }
        }
    
        if (!obstacleDetected) {
            System.out.println("No obstacle detected on the path.");
        }
    
        return detours;
    }
    
    // Calculate the best detour point for a given obstacle edge
    private static Translation2d calculateBestDetour(
        Translation2d start,
        Translation2d end,
        Translation2d vertex1,
        Translation2d vertex2
    ) {
        // Calculate midpoint and trajectory direction
        Translation2d midpoint = vertex1.plus(vertex2).div(2);
        Translation2d trajectoryDirection = end.minus(start);
        double magnitude = trajectoryDirection.getNorm();
        if (magnitude > 0) {
            trajectoryDirection = trajectoryDirection.div(magnitude);
        }
    
        // Perpendicular vector for detour calculation
        Translation2d perpendicular = new Translation2d(-trajectoryDirection.getY(), trajectoryDirection.getX())
            .times(SAFETY_MARGIN);
    
        // Try both perpendicular directions and pick the better one
        Translation2d detourCandidate1 = midpoint.plus(perpendicular);
        Translation2d detourCandidate2 = midpoint.minus(perpendicular);
    
        // Validate detours
        if (!detourIntersectsObstacle(detourCandidate1, start, end, vertex1, vertex2)) {
            return detourCandidate1;
        } else if (!detourIntersectsObstacle(detourCandidate2, start, end, vertex1, vertex2)) {
            return detourCandidate2;
        }
    
        // Return null if no valid detour can be calculated
        return null;
    }
    
    // Check if a detour point intersects the same obstacle or creates new issues
    private static boolean detourIntersectsObstacle(
        Translation2d detour,
        Translation2d start,
        Translation2d end,
        Translation2d vertex1,
        Translation2d vertex2
    ) {
        // Ensure the detour doesn't re-intersect the current obstacle edge
        if (linesIntersect(start, detour, vertex1, vertex2) || linesIntersect(detour, end, vertex1, vertex2)) {
            return true;
        }
    
        // Check detour against all other obstacle edges
        for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
            Translation2d obstacleVertex1 = HEXAGON_VERTICES.get(i);
            Translation2d obstacleVertex2 = HEXAGON_VERTICES.get(i + 1);
    
            if (linesIntersect(start, detour, obstacleVertex1, obstacleVertex2) ||
                linesIntersect(detour, end, obstacleVertex1, obstacleVertex2)) {
                return true;
            }
        }
    
        return false;
    }    
    
    
    private static Translation2d chooseSafeDetour(
        Translation2d detour1,
        Translation2d detour2,
        Translation2d start,
        Translation2d end,
        Translation2d vertex1,
        Translation2d vertex2
    ) {
        boolean detour1Safe = !linesIntersect(start, detour1, vertex1, vertex2) &&
                              !linesIntersect(detour1, end, vertex1, vertex2);
        boolean detour2Safe = !linesIntersect(start, detour2, vertex1, vertex2) &&
                              !linesIntersect(detour2, end, vertex1, vertex2);
    
        // Prioritize the detour closest to the end point that avoids the obstacle
        double detour1Distance = detour1.getDistance(end);
        double detour2Distance = detour2.getDistance(end);
    
        if (detour1Safe && detour2Safe) {
            return detour1Distance < detour2Distance ? detour1 : detour2;
        } else if (detour1Safe) {
            return detour1;
        } else if (detour2Safe) {
            return detour2;
        }
    
        // If both detours are unsafe, return null to indicate failure
        return null;
    }
    

    private static boolean linesIntersect(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        double det = (p2.getX() - p1.getX()) * (q2.getY() - q1.getY()) - (p2.getY() - p1.getY()) * (q2.getX() - q1.getX());
        if (det == 0) return false; // Parallel lines

        double lambda = ((q2.getY() - q1.getY()) * (q2.getX() - p1.getX()) - (q2.getX() - q1.getX()) * (q2.getY() - p1.getY())) / det;
        double gamma = ((p1.getY() - p2.getY()) * (q2.getX() - p1.getX()) + (p2.getX() - p1.getX()) * (q2.getY() - p1.getY())) / det;

        return (0 <= lambda && lambda <= 1) && (0 <= gamma && gamma <= 1);
    }
}
