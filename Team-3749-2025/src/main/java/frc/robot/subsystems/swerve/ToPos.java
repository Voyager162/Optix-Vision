package frc.robot.subsystems.swerve;

import java.util.*;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.*;

public class ToPos {
    private static final double ROBOT_RADIUS = 0.6;
    private static final double SAFE_MARGIN = ROBOT_RADIUS * 1.5;

    // Hexagon vertices with safety margin applied
    private static final List<Translation2d> HEXAGON_VERTICES = List.of(
        new Translation2d(3.668 - SAFE_MARGIN, 3.520 - SAFE_MARGIN),
        new Translation2d(4.5, 3.039 - SAFE_MARGIN),
        new Translation2d(5.332 + SAFE_MARGIN, 3.520 - SAFE_MARGIN),
        new Translation2d(5.332 + SAFE_MARGIN, 4.480 + SAFE_MARGIN),
        new Translation2d(4.5, 4.961 + SAFE_MARGIN),
        new Translation2d(3.668 - SAFE_MARGIN, 4.480 + SAFE_MARGIN)
    );

    public PathPlannerPath generateDynamicPath(Pose2d initialPose, Pose2d finalPose, double maxVelocity, double maxAcceleration, double maxAngularVelocity, double maxAngularAcceleration) {
        if (initialPose == null) {
            throw new IllegalArgumentException("Initial pose cannot be null!");
        }

        List<Waypoint> waypoints = new ArrayList<>();
        System.out.println("Initial Pose: " + initialPose);
        System.out.println("Final Pose: " + finalPose);

        boolean startInsideHexagon = isPointInsideHexagon(initialPose.getTranslation());
        boolean endInsideHexagon = isPointInsideHexagon(finalPose.getTranslation());

        // Handle starting point inside hexagon
        if (startInsideHexagon) {
            Translation2d exitPoint = findClosestSafePoint(initialPose.getTranslation());
            waypoints.add(new Waypoint(initialPose.getTranslation(), exitPoint, exitPoint));
            System.out.println("Start inside hexagon. Exit point: " + exitPoint);
        } else {
            waypoints.add(new Waypoint(initialPose.getTranslation(), initialPose.getTranslation(), initialPose.getTranslation()));
        }

        // Handle ending point inside hexagon
        if (endInsideHexagon) {
            Translation2d entryPoint = findClosestSafePoint(finalPose.getTranslation());
            waypoints.addAll(generateDetourWaypoints(initialPose.getTranslation(), entryPoint));
            waypoints.add(new Waypoint(entryPoint, finalPose.getTranslation(), finalPose.getTranslation()));
            System.out.println("End inside hexagon. Entry point: " + entryPoint);
        } else {
            waypoints.addAll(generateDetourWaypoints(initialPose.getTranslation(), finalPose.getTranslation()));
        }

        waypoints.add(new Waypoint(finalPose.getTranslation(), finalPose.getTranslation(), finalPose.getTranslation()));

        // Debugging the generated waypoints
        System.out.println("Generated Path:");
        for (int i = 0; i < waypoints.size(); i++) {
            System.out.println("Waypoint " + i + ": " + waypoints.get(i).anchor());
        }

        // Ensure the first point matches the robot's starting position
        Translation2d firstWaypoint = waypoints.get(0).anchor();
        if (!firstWaypoint.equals(initialPose.getTranslation())) {
            System.err.println("ERROR: First waypoint does not match robot's starting position! First waypoint: " + firstWaypoint);
        }

        return new PathPlannerPath(
            waypoints,
            new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration),
            null,
            new GoalEndState(0.0, finalPose.getRotation())
        );
    }

    private boolean isPointInsideHexagon(Translation2d point) {
        int intersectCount = 0;
        Translation2d farPoint = new Translation2d(10000, point.getY());

        for (int i = 0; i < HEXAGON_VERTICES.size(); i++) {
            Translation2d vertex1 = HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = HEXAGON_VERTICES.get((i + 1) % HEXAGON_VERTICES.size());
            if (doLinesIntersect(point, farPoint, vertex1, vertex2)) {
                intersectCount++;
            }
        }
        boolean inside = intersectCount % 2 == 1;
        System.out.println("Point " + point + " inside hexagon: " + inside);
        return inside;
    }

    private boolean doLinesIntersect(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        double det = (q2.getX() - q1.getX()) * (p2.getY() - p1.getY()) - (q2.getY() - q1.getY()) * (p2.getX() - p1.getX());
        if (det == 0) return false;

        double t = ((q1.getY() - p1.getY()) * (q2.getX() - q1.getX()) - (q1.getX() - p1.getX()) * (q2.getY() - q1.getY())) / det;
        double u = ((q1.getY() - p1.getY()) * (p2.getX() - p1.getX()) - (q1.getX() - p1.getX()) * (p2.getY() - p1.getY())) / det;

        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private Translation2d findClosestSafePoint(Translation2d point) {
        Translation2d closest = null;
        double minDistance = Double.MAX_VALUE;

        System.out.println("Finding closest safe point for: " + point);

        for (Translation2d vertex : HEXAGON_VERTICES) {
            double distance = point.getDistance(vertex);
            System.out.println("Checking vertex: " + vertex + ", Distance: " + distance);

            if (distance < minDistance) {
                minDistance = distance;
                closest = vertex;
                System.out.println("New closest vertex: " + closest + ", Distance: " + minDistance);
            }
        }

        if (closest == null) {
            System.err.println("ERROR: No closest vertex found! This shouldn't happen.");
            return point; // Fallback to original point if something goes wrong.
        }

        double distanceToVertex = point.getDistance(closest);
        double clampedMargin = Math.min(SAFE_MARGIN, distanceToVertex * 0.5); // Limit the offset
        double angle = Math.atan2(closest.getY() - point.getY(), closest.getX() - point.getX());
        double safeX = closest.getX() + clampedMargin * Math.cos(angle);
        double safeY = closest.getY() + clampedMargin * Math.sin(angle);

        Translation2d safePoint = new Translation2d(safeX, safeY);

        // Log detailed information
        System.out.println("Closest vertex: " + closest);
        System.out.println("Angle to closest vertex: " + Math.toDegrees(angle) + " degrees");
        System.out.println("Calculated safe point: " + safePoint);

        return safePoint;
    }

    private boolean isPathIntersectingObstacle(Translation2d start, Translation2d end) {
        for (int i = 0; i < HEXAGON_VERTICES.size(); i++) {
            Translation2d vertex1 = HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = HEXAGON_VERTICES.get((i + 1) % HEXAGON_VERTICES.size());
            if (doLinesIntersect(start, end, vertex1, vertex2)) {
                System.out.println("Path from " + start + " to " + end + " intersects obstacle.");
                return true;
            }
        }
        return false;
    }

    private List<Waypoint> generateDetourWaypoints(Translation2d start, Translation2d end) {
        List<Waypoint> detourWaypoints = new ArrayList<>();
        if (!isPathIntersectingObstacle(start, end)) {
            detourWaypoints.add(new Waypoint(start, start, end));
            System.out.println("Direct path from " + start + " to " + end + " is safe.");
            return detourWaypoints;
        }

        int startVertexIndex = findClosestVertexIndex(start);
        int endVertexIndex = findClosestVertexIndex(end);

        int clockwiseDistance = (endVertexIndex - startVertexIndex + HEXAGON_VERTICES.size()) % HEXAGON_VERTICES.size();
        int counterclockwiseDistance = (startVertexIndex - endVertexIndex + HEXAGON_VERTICES.size()) % HEXAGON_VERTICES.size();

        boolean goClockwise = clockwiseDistance < counterclockwiseDistance;

        System.out.println("Generating detour waypoints from " + start + " to " + end);
        System.out.println("Clockwise: " + clockwiseDistance + ", Counterclockwise: " + counterclockwiseDistance);

        int currentIndex = startVertexIndex;
        while (currentIndex != endVertexIndex) {
            Translation2d vertex = HEXAGON_VERTICES.get(currentIndex);
            detourWaypoints.add(new Waypoint(vertex, vertex, vertex));
            System.out.println("Adding detour waypoint: " + vertex);
            currentIndex = goClockwise
                ? (currentIndex + 1) % HEXAGON_VERTICES.size()
                : (currentIndex - 1 + HEXAGON_VERTICES.size()) % HEXAGON_VERTICES.size();
        }

        detourWaypoints.add(new Waypoint(HEXAGON_VERTICES.get(endVertexIndex), end, end));
        System.out.println("Final detour waypoint: " + HEXAGON_VERTICES.get(endVertexIndex));
        return detourWaypoints;
    }

    private int findClosestVertexIndex(Translation2d point) {
        int closestIndex = -1;
        double minDistance = Double.MAX_VALUE;

        for (int i = 0; i < HEXAGON_VERTICES.size(); i++) {
            double distance = point.getDistance(HEXAGON_VERTICES.get(i));
            if (distance < minDistance) {
                minDistance = distance;
                closestIndex = i;
            }
        }

        System.out.println("Closest vertex to " + point + ": " + HEXAGON_VERTICES.get(closestIndex));
        return closestIndex;
    }
}
