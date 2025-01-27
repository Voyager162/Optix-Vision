package frc.robot.subsystems.swerve;

import java.util.*;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.*;

public class ToPos {
    private static final double ROBOT_RADIUS = 0.9;
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
        boolean startInsideHexagon = isPointInsideHexagon(initialPose.getTranslation());
        boolean endInsideHexagon = isPointInsideHexagon(finalPose.getTranslation());

        List<Waypoint> waypoints = new ArrayList<>();

        // Handle if the start point is inside the hexagon
        if (startInsideHexagon) {
            Translation2d exitPoint = findClosestSafePoint(initialPose.getTranslation());
            waypoints.add(new Waypoint(initialPose.getTranslation(), exitPoint, exitPoint));
        }

        // Handle detours around the hexagon
        if (endInsideHexagon) {
            Translation2d entryPoint = findClosestSafePoint(finalPose.getTranslation());
            waypoints.addAll(generateDetourWaypoints(initialPose.getTranslation(), entryPoint));
            waypoints.add(new Waypoint(entryPoint, finalPose.getTranslation(), finalPose.getTranslation()));
        } else {
            waypoints.addAll(generateDetourWaypoints(initialPose.getTranslation(), finalPose.getTranslation()));
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
        Translation2d farPoint = new Translation2d(10000, point.getY()); // A far-off point for ray-casting

        for (int i = 0; i < HEXAGON_VERTICES.size(); i++) {
            Translation2d vertex1 = HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = HEXAGON_VERTICES.get((i + 1) % HEXAGON_VERTICES.size());
            if (doLinesIntersect(point, farPoint, vertex1, vertex2)) {
                intersectCount++;
            }
        }
        return intersectCount % 2 == 1; // Odd intersection count means the point is inside
    }

    private boolean doLinesIntersect(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        double det = (q2.getX() - q1.getX()) * (p2.getY() - p1.getY()) - (q2.getY() - q1.getY()) * (p2.getX() - p1.getX());
        if (det == 0) return false; // Parallel lines

        double t = ((q1.getY() - p1.getY()) * (q2.getX() - q1.getX()) - (q1.getX() - p1.getX()) * (q2.getY() - q1.getY())) / det;
        double u = ((q1.getY() - p1.getY()) * (p2.getX() - p1.getX()) - (q1.getX() - p1.getX()) * (p2.getY() - p1.getY())) / det;

        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private Translation2d findClosestSafePoint(Translation2d point) {
        Translation2d closest = null;
        double minDistance = Double.MAX_VALUE;

        for (Translation2d vertex : HEXAGON_VERTICES) {
            double distance = point.getDistance(vertex);
            if (distance < minDistance) {
                minDistance = distance;
                closest = vertex;
            }
        }

        // Offset slightly outside the hexagon
        double angle = Math.atan2(closest.getY() - point.getY(), closest.getX() - point.getX());
        return new Translation2d(
            closest.getX() + SAFE_MARGIN * Math.cos(angle),
            closest.getY() + SAFE_MARGIN * Math.sin(angle)
        );
    }

    private List<Waypoint> generateDetourWaypoints(Translation2d start, Translation2d end) {
        List<Waypoint> detourWaypoints = new ArrayList<>();
        int startVertexIndex = findClosestVertexIndex(start);
        int endVertexIndex = findClosestVertexIndex(end);

        // Traverse along hexagon edges between closest vertices
        int currentIndex = startVertexIndex;
        while (currentIndex != endVertexIndex) {
            Translation2d vertex = HEXAGON_VERTICES.get(currentIndex);
            detourWaypoints.add(new Waypoint(vertex, vertex, vertex));
            currentIndex = (currentIndex + 1) % HEXAGON_VERTICES.size();
        }

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

        return closestIndex;
    }
}
