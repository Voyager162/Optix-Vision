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
    private static final double ROBOT_RADIUS = 0.8382; // Robot's radius in meters
    private static final double INITIAL_SAFETY_MARGIN = 1.0;
    private static final double SAFETY_MARGIN_INCREMENT = 0.2;
    private static final double MIN_CLEARANCE = 0.5; // Clearance for obstacles
    private static final double FINAL_APPROACH_DISTANCE = 1.0;
    private static final double MIN_SPEED = 0.5; // Minimum speed to avoid excessive slowing
    private static final List<Translation2d> HEXAGON_VERTICES = List.of(
        new Translation2d(3.76, 3.49),
        new Translation2d(3.66, 4.39),
        new Translation2d(4.39, 4.93),
        new Translation2d(5.22, 4.57),
        new Translation2d(5.32, 3.67),
        new Translation2d(4.59, 3.13),
        new Translation2d(3.76, 3.49)
    );

    private static final ExecutorService executorService = Executors.newSingleThreadExecutor();

    public static void generateDynamicPathAsync(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration
    ) {
        executorService.submit(() -> {
            System.out.println("Starting path generation...");
            PathPlannerPath path = generateDynamicPath(
                initialPose, finalPose, maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration
            );
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
            double maxAngularAcceleration
    ) {
        List<Waypoint> waypoints = new ArrayList<>();

        // Use the exact robot position as the starting point
        Translation2d start = adjustIfTooCloseToObstacle(initialPose.getTranslation());
        Translation2d end = finalPose.getTranslation();
        Translation2d finalApproach = calculateFinalApproachPoint(end, finalPose.getRotation());

        waypoints.add(new Waypoint(start, start, start));

        // Add detours dynamically if obstacles are detected
        List<Translation2d> detours = calculateDetour(start, finalApproach);
        for (Translation2d detour : detours) {
            waypoints.add(new Waypoint(detour, detour, detour));
        }

        waypoints.add(new Waypoint(finalApproach, finalApproach, finalApproach));
        waypoints.add(new Waypoint(end, end, end));

        if (waypoints.size() < 2) {
            System.out.println("Error: Not enough waypoints for path generation.");
            return null;
        }

        // Path constraints with a minimum speed to avoid excessive slowing
        PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);

        try {
            Rotation2d fullRotation = calculateFullRotation(initialPose.getRotation(), finalPose.getRotation());
            PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, fullRotation));
            path.preventFlipping = true;
            return path;
        } catch (Exception e) {
            System.out.println("PathPlanner Error: " + e.getMessage());
            return null;
        }
    }

    private static Translation2d adjustIfTooCloseToObstacle(Translation2d point) {
        for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
            Translation2d vertex1 = HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = HEXAGON_VERTICES.get(i + 1);

            if (isPointNearEdge(point, vertex1, vertex2)) {
                System.out.println("Point too close to obstacle edge! Adjusting...");
                return movePointAwayFromEdge(point, vertex1, vertex2);
            }
        }
        return point;
    }

    private static Translation2d calculateFinalApproachPoint(Translation2d end, Rotation2d finalHeading) {
        double dx = FINAL_APPROACH_DISTANCE * Math.cos(finalHeading.getRadians());
        double dy = FINAL_APPROACH_DISTANCE * Math.sin(finalHeading.getRadians());
        return new Translation2d(end.getX() - dx, end.getY() - dy);
    }

    private static boolean isPointNearEdge(Translation2d point, Translation2d vertex1, Translation2d vertex2) {
        Translation2d edge = vertex2.minus(vertex1);
        double edgeLength = edge.getNorm();
        double projection = Math.max(0, Math.min(1, 
            ((point.getX() - vertex1.getX()) * edge.getX() + 
             (point.getY() - vertex1.getY()) * edge.getY()) / edgeLength));
        Translation2d closestPoint = vertex1.plus(edge.times(projection));
        return point.getDistance(closestPoint) < ROBOT_RADIUS + MIN_CLEARANCE;
    }

    private static Translation2d movePointAwayFromEdge(Translation2d point, Translation2d vertex1, Translation2d vertex2) {
        Translation2d edge = vertex2.minus(vertex1).div(vertex2.minus(vertex1).getNorm());
        Translation2d normal = new Translation2d(-edge.getY(), edge.getX());
        return point.plus(normal.times(ROBOT_RADIUS + MIN_CLEARANCE));
    }

    private static List<Translation2d> calculateDetour(Translation2d start, Translation2d end) {
        List<Translation2d> detours = new ArrayList<>();
        double safetyMargin = INITIAL_SAFETY_MARGIN;

        for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
            Translation2d vertex1 = HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = HEXAGON_VERTICES.get(i + 1);

            if (linesIntersect(start, end, vertex1, vertex2)) {
                System.out.println("Obstacle detected! Intersecting edge: " + vertex1 + " to " + vertex2);

                while (safetyMargin < 5.0) {
                    Translation2d midpoint = vertex1.plus(vertex2).div(2);
                    Translation2d trajectoryDirection = end.minus(start).div(end.getDistance(start));
                    Translation2d perpendicular = new Translation2d(-trajectoryDirection.getY(), trajectoryDirection.getX()).times(safetyMargin);

                    Translation2d detour = midpoint.plus(perpendicular);
                    if (isValidDetour(detour, start, end)) {
                        detours.add(detour);

                        // Add an additional intermediate detour to smooth the path
                        Translation2d intermediateDetour = detour.plus(perpendicular.times(0.5));
                        if (isValidDetour(intermediateDetour, detour, end)) {
                            detours.add(intermediateDetour);
                        }
                        return detours;
                    }

                    safetyMargin += SAFETY_MARGIN_INCREMENT;
                }

                System.out.println("Failed to calculate a valid detour.");
                break;
            }
        }
        return detours;
    }

    private static boolean linesIntersect(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        double det = (p2.getX() - p1.getX()) * (q2.getY() - q1.getY()) - (p2.getY() - p1.getY()) * (q2.getX() - q1.getX());
        if (det == 0) return false;

        double lambda = ((q2.getY() - q1.getY()) * (q2.getX() - p1.getX()) - (q2.getX() - q1.getX()) * (q2.getY() - p1.getY())) / det;
        double gamma = ((p1.getY() - p2.getY()) * (q2.getX() - p1.getX()) + (p2.getX() - p1.getX()) * (q2.getY() - p1.getY())) / det;

        return (0 <= lambda && lambda <= 1) && (0 <= gamma && gamma <= 1);
    }

    private static boolean isValidDetour(Translation2d detour, Translation2d start, Translation2d end) {
        for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
            Translation2d vertex1 = HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = HEXAGON_VERTICES.get(i + 1);

            if (linesIntersect(start, detour, vertex1, vertex2) ||
                linesIntersect(detour, end, vertex1, vertex2)) {
                return false;
            }
        }
        return true;
    }

    private static Rotation2d calculateFullRotation(Rotation2d start, Rotation2d end) {
        double startRadians = start.getRadians();
        double endRadians = end.getRadians();
        double delta = endRadians - startRadians;

        if (delta > Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;

        return new Rotation2d(startRadians + delta);
    }
}
