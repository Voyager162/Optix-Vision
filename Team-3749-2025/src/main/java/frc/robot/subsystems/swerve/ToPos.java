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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ToPos {
    private static final double ROBOT_RADIUS = 0.8382 / 2; // Robot's radius in meters
    private static final double CLEARANCE_OFFSET = ROBOT_RADIUS * 1.5; // Clearance offset multiplier
    private static final List<Translation2d> HEXAGON_VERTICES = List.of(
            new Translation2d(3.668 - ROBOT_RADIUS / 2 * 1.732, 3.520 - ROBOT_RADIUS / 2),
            new Translation2d(4.5, 3.039 - ROBOT_RADIUS),
            new Translation2d(5.332 + ROBOT_RADIUS / 2 * 1.732, 3.520 - ROBOT_RADIUS / 2),
            new Translation2d(5.332 + ROBOT_RADIUS / 2 * 1.732, 4.480 + ROBOT_RADIUS / 2),
            new Translation2d(4.5, 4.961 + ROBOT_RADIUS),
            new Translation2d(3.668 - ROBOT_RADIUS / 2 * 1.732, 4.480 + ROBOT_RADIUS / 2),
            new Translation2d(3.668 - ROBOT_RADIUS / 2 * 1.732, 3.520 - ROBOT_RADIUS / 2)); // Closed loop

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
        List<Waypoint> waypoints = new ArrayList<>();

        // Adjust the start point to ensure it's outside the obstacle
        Translation2d start = adjustWaypointOutsideObstacle(initialPose.getTranslation(), initialPose.getRotation(),
                true);
        Translation2d end = adjustWaypointOutsideObstacle(finalPose.getTranslation(), finalPose.getRotation(), false);

        // Add the adjusted start position
        waypoints.add(new Waypoint(start, start, start));

        // Calculate detours dynamically
        List<Translation2d> detours = calculateDetour(start, end);
        for (Translation2d detour : detours) {
            waypoints.add(new Waypoint(detour, detour, detour));
        }

        // Add the final waypoint
        waypoints.add(new Waypoint(end, end, end));

        // Validate waypoints
        if (waypoints.size() < 2) {
            System.out.println("Error: Not enough waypoints for path generation.");
            return null;
        }

        // Path constraints
        PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity,
                maxAngularAcceleration);

        try {
            // Generate path
            Rotation2d fullRotation = calculateFullRotation(initialPose.getRotation(), finalPose.getRotation());
            PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null,
                    new GoalEndState(0.0, fullRotation));
            path.preventFlipping = true;
            return path;
        } catch (Exception e) {
            System.out.println("PathPlanner Error: " + e.getMessage());
            return null;
        }
    }

    private static Translation2d adjustWaypointOutsideObstacle(Translation2d point, Rotation2d heading,
            boolean isStart) {
        if (!isInsideObstacle(point)) {
            return point; // Already outside
        }

        double step = 0.1; // Step size for moving outward
        Translation2d adjustedPoint = point;
        double dx = (isStart ? 1 : -1) * step * Math.cos(heading.getRadians());
        double dy = (isStart ? 1 : -1) * step * Math.sin(heading.getRadians());

        // Move outward until the point is outside the obstacle
        while (isInsideObstacle(adjustedPoint)) {
            adjustedPoint = new Translation2d(adjustedPoint.getX() + dx, adjustedPoint.getY() + dy);
        }

        return adjustedPoint;
    }

    private static boolean isInsideObstacle(Translation2d point) {
        int intersections = 0;
        Translation2d rayEnd = new Translation2d(1e9, point.getY()); // A faraway point for ray casting

        for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
            Translation2d hexStart = HEXAGON_VERTICES.get(i);
            Translation2d hexEnd = HEXAGON_VERTICES.get(i + 1);

            if (findIntersection(point, rayEnd, hexStart, hexEnd) != null) {
                intersections++;
            }
        }

        return intersections % 2 == 1;

    }

    private static List<Translation2d> detectObstacleIntersections(Translation2d start, Translation2d end) {
        List<Translation2d> intersectionPoints = new ArrayList<>();

        // Check each edge of the obstacle
        for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
            Translation2d hexStart = HEXAGON_VERTICES.get(i);
            Translation2d hexEnd = HEXAGON_VERTICES.get(i + 1);

            Translation2d intersection = findIntersection(start, end, hexStart, hexEnd);
            if (intersection != null) {
                intersectionPoints.add(intersection);
                System.out.println("Intersection detected at: " + intersection);
            }
        }

        // Sort intersections by distance from the start
        intersectionPoints.sort((a, b) -> Double.compare(a.getDistance(start), b.getDistance(start)));

        return intersectionPoints;
    }

    private static List<Translation2d> calculateDetour(Translation2d start, Translation2d end) {
        List<Translation2d> detours = new ArrayList<>();

        // Detect intersections with the obstacle
        List<Translation2d> intersections = detectObstacleIntersections(start, end);

        if (intersections.isEmpty()) {
            System.out.println("No intersections detected. No detours needed.");
            return detours;
        }

        // Iterate through each intersection and add clearance points
        for (Translation2d intersection : intersections) {
            Translation2d clearancePoint = calculateClearancePoint(intersection, start, end);

            // Verify that the clearance point does not intersect the obstacle
            if (!isInsideObstacle(clearancePoint)) {
                detours.add(clearancePoint);
                System.out.println("Clearance point added: " + clearancePoint);
            } else {
                System.out.println("Clearance point is inside the obstacle. Adjusting further.");
                detours.add(adjustPointOutsideObstacle(clearancePoint, start, end));
            }
        }

        return detours;
    }

    private static Translation2d calculateClearancePoint(Translation2d intersection, Translation2d start,
            Translation2d end) {
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double length = Math.sqrt(dx * dx + dy * dy);

        // Offset perpendicular to the path
        double offsetX = -dy / length * CLEARANCE_OFFSET;
        double offsetY = dx / length * CLEARANCE_OFFSET;

        return new Translation2d(intersection.getX() + offsetX, intersection.getY() + offsetY);
    }

    private static Translation2d adjustPointOutsideObstacle(Translation2d point, Translation2d start,
            Translation2d end) {
        double step = 0.1; // Small step size for adjustments
        Translation2d adjustedPoint = point;

        // Adjust the point outward until it's outside the obstacle
        while (isInsideObstacle(adjustedPoint)) {
            double dx = end.getX() - start.getX();
            double dy = end.getY() - start.getY();
            double length = Math.sqrt(dx * dx + dy * dy);

            // Move outward along the perpendicular direction
            double offsetX = -dy / length * step;
            double offsetY = dx / length * step;
            adjustedPoint = new Translation2d(adjustedPoint.getX() + offsetX, adjustedPoint.getY() + offsetY);
        }

        return adjustedPoint;
    }

    private static Translation2d findIntersection(Translation2d p1, Translation2d p2, Translation2d q1,
            Translation2d q2) {
        double det = (p2.getX() - p1.getX()) * (q2.getY() - q1.getY())
                - (p2.getY() - p1.getY()) * (q2.getX() - q1.getX());

        if (det == 0) {
            // Lines are parallel or coincident
            System.out.println("No intersection: Lines are parallel.");
            return null;
        }

        double lambda = ((q2.getY() - q1.getY()) * (q2.getX() - p1.getX())
                - (q2.getX() - q1.getX()) * (q2.getY() - p1.getY())) / det;
        double gamma = ((p1.getY() - p2.getY()) * (q2.getX() - p1.getX())
                + (p2.getX() - p1.getX()) * (q2.getY() - p1.getY())) / det;

        if ((0 <= lambda && lambda <= 1) && (0 <= gamma && gamma <= 1)) {
            // Intersection exists within the segments
            double x = p1.getX() + lambda * (p2.getX() - p1.getX());
            double y = p1.getY() + lambda * (p2.getY() - p1.getY());
            System.out.println("Intersection found at: (" + x + ", " + y + ")");
            return new Translation2d(x, y);
        }

        System.out.println("No intersection: Lines do not intersect within segments.");
        return null;
    }

    private static Rotation2d calculateFullRotation(Rotation2d start, Rotation2d end) {
        double startRadians = start.getRadians();
        double endRadians = end.getRadians();
        double delta = endRadians - startRadians;

        if (delta > Math.PI)
            delta -= 2 * Math.PI;
        if (delta < -Math.PI)
            delta += 2 * Math.PI;

        return new Rotation2d(startRadians + delta);

    }
}
