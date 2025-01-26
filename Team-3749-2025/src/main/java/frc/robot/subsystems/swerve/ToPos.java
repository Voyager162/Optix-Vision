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
    private static final double ROBOT_RADIUS = 0.9; // Robot's radius in meters
    private static final double robRadY = ROBOT_RADIUS / 2;
    private static final double robRadX = robRadY * 1.7320508076;
    private static final List<Translation2d> HEXAGON_VERTICES = List.of(
            new Translation2d(3.668 - robRadX, 3.520 - robRadY),
            new Translation2d(4.5, 3.039 - ROBOT_RADIUS),
            new Translation2d(5.332 + robRadX, 3.520 - robRadY),
            new Translation2d(5.332 + robRadX, 4.480 + robRadY),
            new Translation2d(4.5, 4.961 + ROBOT_RADIUS),
            new Translation2d(3.668 - robRadX, 4.480 + robRadY),
            new Translation2d(3.668 - robRadX, 3.520 - robRadY));

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

        // Adjust the start point if it is inside the obstacle
        Translation2d start = adjustWaypointOutsideObstacle(initialPose.getTranslation(), initialPose.getRotation());

        // Add the adjusted start position
        waypoints.add(new Waypoint(start, start, start));

        // Calculate detour points
        List<Translation2d> detours = calculateDetour(start, finalPose.getTranslation());
        for (Translation2d detour : detours) {
            waypoints.add(new Waypoint(detour, detour, detour));
        }

        // Add the final destination (always within the obstacle if necessary)
        Translation2d end = finalPose.getTranslation();
        waypoints.add(new Waypoint(end, end, end));

        // Ensure there are enough waypoints for path generation
        if (waypoints.size() < 2) {
            System.out.println("Error: Not enough waypoints for path generation.");
            return null;
        }

        // Create path constraints
        PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity,
                maxAngularAcceleration);

        try {
            // Generate the path
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

    private static Translation2d adjustWaypointOutsideObstacle(Translation2d point, Rotation2d heading) {
        if (!isInsideObstacle(point)) {
            return point; // Already outside
        }

        double step = 0.1; // Step size for moving outward
        Translation2d adjustedPoint = point;
        double dx = -step * Math.cos(heading.getRadians());
        double dy = -step * Math.sin(heading.getRadians());

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

        // Odd number of intersections => inside, even => outside
        return intersections % 2 == 1;
    }

    private static List<Translation2d> calculateDetour(Translation2d start, Translation2d end) {
        List<Translation2d> detours = new ArrayList<>();
        boolean intersects = false;

        for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
            Translation2d hexStart = HEXAGON_VERTICES.get(i);
            Translation2d hexEnd = HEXAGON_VERTICES.get(i + 1);

            if (findIntersection(start, end, hexStart, hexEnd) != null) {
                intersects = true;
                break;
            }
        }

        // If no intersections, no detour is needed
        if (!intersects) {
            return detours;
        }

        // Add all obstacle vertices for detouring
        detours.addAll(HEXAGON_VERTICES);

        return detours;
    }

    private static Translation2d findIntersection(Translation2d p1, Translation2d p2, Translation2d q1,
            Translation2d q2) {
        double det = (p2.getX() - p1.getX()) * (q2.getY() - q1.getY())
                - (p2.getY() - p1.getY()) * (q2.getX() - q1.getX());
        if (det == 0) {
            return null; // Parallel lines
        }

        double lambda = ((q2.getY() - q1.getY()) * (q2.getX() - p1.getX())
                - (q2.getX() - q1.getX()) * (q2.getY() - p1.getY())) / det;
        double gamma = ((p1.getY() - p2.getY()) * (q2.getX() - p1.getX())
                + (p2.getX() - p1.getX()) * (q2.getY() - p1.getY())) / det;

        if ((0 <= lambda && lambda <= 1) && (0 <= gamma && gamma <= 1)) {
            double x = p1.getX() + lambda * (p2.getX() - p1.getX());
            double y = p1.getY() + lambda * (p2.getY() - p1.getY());
            return new Translation2d(x, y);
        }

        return null; // No intersection
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
