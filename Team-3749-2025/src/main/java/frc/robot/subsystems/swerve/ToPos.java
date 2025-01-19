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
            Pose2d beforeFinalPose,
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
            beforeFinalPose.getTranslation(),
            beforeFinalPose.getTranslation(),
            finalPose.getTranslation()
        ));

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
        boolean obstacleDetected = false;

        for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
            Translation2d vertex1 = HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = HEXAGON_VERTICES.get(i + 1);

            if (linesIntersect(start, end, vertex1, vertex2)) {
                obstacleDetected = true;
                System.out.println("Obstacle detected! Intersecting edge: " + vertex1 + " to " + vertex2);

                // Calculate detour points to avoid the obstacle
                Translation2d midpoint = vertex1.plus(vertex2).div(2);
                Translation2d perpendicular = new Translation2d(
                    -(end.getY() - start.getY()), end.getX() - start.getX()
                ).div(start.getDistance(end)).times(SAFETY_MARGIN + 0.5); // Ensure perpendicular detour
                Translation2d detour1 = midpoint.plus(perpendicular);
                Translation2d detour2 = midpoint.minus(perpendicular);

                // Choose the detour point closer to the trajectory's direction
                double detour1Distance = detour1.getDistance(end);
                double detour2Distance = detour2.getDistance(end);
                detours.add(detour1Distance < detour2Distance ? detour1 : detour2);

                System.out.println("Added detour waypoint: " + (detour1Distance < detour2Distance ? detour1 : detour2));
            }
        }

        if (!obstacleDetected) {
            System.out.println("No obstacle detected on the path.");
        }

        return detours;
    }

    private static boolean linesIntersect(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        double det = (p2.getX() - p1.getX()) * (q2.getY() - q1.getY()) - (p2.getY() - p1.getY()) * (q2.getX() - q1.getX());
        if (det == 0) return false; // Parallel lines

        double lambda = ((q2.getY() - q1.getY()) * (q2.getX() - p1.getX()) - (q2.getX() - q1.getX()) * (q2.getY() - p1.getY())) / det;
        double gamma = ((p1.getY() - p2.getY()) * (q2.getX() - p1.getX()) + (p2.getX() - p1.getX()) * (q2.getY() - p1.getY())) / det;

        return (0 <= lambda && lambda <= 1) && (0 <= gamma && gamma <= 1);
    }
}
