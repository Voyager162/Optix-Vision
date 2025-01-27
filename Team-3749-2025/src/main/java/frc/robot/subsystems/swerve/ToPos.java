package frc.robot.subsystems.swerve;

import java.util.*;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.*;

/**
 * This class generates dynamic paths for a robot to move from one pose to another
 * while avoiding obstacles, specifically a hexagonal area defined by safety margins.
 */
public class ToPos {

    // MAKE CONSTANTS 
    private static final double SAFE_MARGIN = 1; // Safety margin around the robot.

    // Vertices of the hexagon, adjusted for safety margins.
    private static final List<Translation2d> HEXAGON_VERTICES = List.of(
        new Translation2d(3.668 - SAFE_MARGIN, 3.520 - SAFE_MARGIN),
        new Translation2d(4.5, 3.039 - SAFE_MARGIN),
        new Translation2d(5.332 + SAFE_MARGIN, 3.520 - SAFE_MARGIN),
        new Translation2d(5.332 + SAFE_MARGIN, 4.480 + SAFE_MARGIN),
        new Translation2d(4.5, 4.961 + SAFE_MARGIN),
        new Translation2d(3.668 - SAFE_MARGIN, 4.480 + SAFE_MARGIN)
    );

    /**
     * Generates a dynamic path for the robot from an initial pose to a final pose.
     *
     * @param initialPose          The starting position and orientation of the robot.
     * @param finalPose            The target position and orientation of the robot.
     * @param maxVelocity          Maximum velocity of the robot (m/s).
     * @param maxAcceleration      Maximum acceleration of the robot (m/s^2).
     * @param maxAngularVelocity   Maximum angular velocity of the robot (rad/s).
     * @param maxAngularAcceleration Maximum angular acceleration of the robot (rad/s^2).
     * @return                     A PathPlannerPath containing waypoints for the robot to follow.
     */
    public PathPlannerPath generateDynamicPath(Pose2d initialPose,Pose2d approachPoint, Pose2d finalPose, double maxVelocity, double maxAcceleration, double maxAngularVelocity, double maxAngularAcceleration) {
        if (initialPose == null) {
            throw new IllegalArgumentException("Initial pose cannot be null!");
        }

        List<Waypoint> waypoints = new ArrayList<>();

        // Check if the start or end points are inside the hexagonal obstacle.
        boolean startInsideHexagon = isPointInsideHexagon(initialPose.getTranslation());
        boolean endInsideHexagon = isPointInsideHexagon(finalPose.getTranslation());

        // Handle starting point inside the hexagon.
        if (startInsideHexagon) {
            Translation2d exitPoint = findClosestSafePointWithHeading(initialPose.getTranslation(), initialPose.getRotation(), false);
            waypoints.add(new Waypoint(initialPose.getTranslation(), exitPoint, exitPoint));
        } else {
            waypoints.add(new Waypoint(initialPose.getTranslation(), initialPose.getTranslation(), initialPose.getTranslation()));
        }
        // Handle ending point inside the hexagon.
        if (endInsideHexagon) {
            Translation2d entryPoint = findClosestSafePointWithHeading(finalPose.getTranslation(), finalPose.getRotation(), true);
            waypoints.addAll(generateDetourWaypoints(initialPose.getTranslation(), entryPoint));
            waypoints.add(new Waypoint(approachPoint.getTranslation(),approachPoint.getTranslation(), approachPoint.getTranslation()));
            waypoints.add(new Waypoint(entryPoint, finalPose.getTranslation(), finalPose.getTranslation()));
        } else {
            waypoints.addAll(generateDetourWaypoints(initialPose.getTranslation(), finalPose.getTranslation()));
            waypoints.add(new Waypoint(approachPoint.getTranslation(),approachPoint.getTranslation(), approachPoint.getTranslation()));
        }

        waypoints.add(new Waypoint(finalPose.getTranslation(), finalPose.getTranslation(), finalPose.getTranslation()));

        // Remove redundant waypoints to prevent small loops.
        removeRedundantWaypoints(waypoints);

        return new PathPlannerPath(
            waypoints,
            new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration),
            null,
            new GoalEndState(0.0, finalPose.getRotation())
        );
    }

    /**
     * Removes redundant waypoints that are too close to each other.
     *
     * @param waypoints The list of waypoints to be cleaned.
     */
    private void removeRedundantWaypoints(List<Waypoint> waypoints) {
        double threshold = 0.05; // Minimum distance between waypoints to consider them unique (5 cm).
        for (int i = 1; i < waypoints.size(); i++) {
            Translation2d current = waypoints.get(i).anchor();
            Translation2d previous = waypoints.get(i - 1).anchor();
            if (current.getDistance(previous) < threshold) {
                waypoints.remove(i);
                i--; // Adjust index after removal.
            }
        }
    }

    /**
     * Checks if a given point lies inside the hexagon.
     *
     * @param point The point to check.
     * @return      True if the point is inside the hexagon, false otherwise.
     */
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

        return intersectCount % 2 == 1;
    }

    /**
     * Determines if two line segments intersect.
     *
     * @param p1 First point of the first line segment.
     * @param p2 Second point of the first line segment.
     * @param q1 First point of the second line segment.
     * @param q2 Second point of the second line segment.
     * @return   True if the lines intersect, false otherwise.
     */
    private boolean doLinesIntersect(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        double det = (q2.getX() - q1.getX()) * (p2.getY() - p1.getY()) - (q2.getY() - q1.getY()) * (p2.getX() - p1.getX());
        if (det == 0) return false;

        double t = ((q1.getY() - p1.getY()) * (q2.getX() - q1.getX()) - (q1.getX() - p1.getX()) * (q2.getY() - q1.getY())) / det;
        double u = ((q1.getY() - p1.getY()) * (p2.getX() - p1.getX()) - (q1.getX() - p1.getX()) * (p2.getY() - p1.getY())) / det;

        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    /**
     * Finds the closest safe point to a given point, based on the robot's heading.
     *
     * @param point       The current point.
     * @param heading     The robot's orientation.
     * @param isFinalPoint Whether the point is the final destination.
     * @return            The closest safe point.
     */
    private Translation2d findClosestSafePointWithHeading(Translation2d point, Rotation2d heading, boolean isFinalPoint) {
        Translation2d closest = null;
        double minDistance = Double.MAX_VALUE;
    
        // Find the closest vertex
        for (Translation2d vertex : HEXAGON_VERTICES) {
            double distance = point.getDistance(vertex);
            if (distance < minDistance) {
                minDistance = distance;
                closest = vertex;
            }
        }
    
        if (closest == null) {
            return point; // Fallback to original point if no safe point is found.
        }
    
        double safeX, safeY;
    
        if (isFinalPoint) {
            // For the final point, use the heading to move away from the point
            safeX = point.getX() - Math.cos(heading.getRadians());
            safeY = point.getY() - Math.sin(heading.getRadians());
        } else {
            // For other points, apply a clamped margin and adjust the point based on heading
            double clampedMargin = Math.min(SAFE_MARGIN, point.getDistance(closest) * 0.5);
            safeX = point.getX() - clampedMargin * Math.cos(heading.getRadians());
            safeY = point.getY() - clampedMargin * Math.sin(heading.getRadians());
        }
    
        return new Translation2d(safeX, safeY);
    }
    
    /**
     * Generates detour waypoints to avoid obstacles between two points.
     *
     * @param start The starting point.
     * @param end   The target point.
     * @return      A list of detour waypoints.
     */
    private List<Waypoint> generateDetourWaypoints(Translation2d start, Translation2d end) {
        List<Waypoint> detourWaypoints = new ArrayList<>();

        if (!isPathIntersectingObstacle(start, end)) {
            detourWaypoints.add(new Waypoint(start, start, end));
            return detourWaypoints;
        }

        int startVertexIndex = findClosestVertexIndex(start);
        int endVertexIndex = findClosestVertexIndex(end);

        int clockwiseDistance = (endVertexIndex - startVertexIndex + HEXAGON_VERTICES.size()) % HEXAGON_VERTICES.size();
        int counterclockwiseDistance = (startVertexIndex - endVertexIndex + HEXAGON_VERTICES.size()) % HEXAGON_VERTICES.size();

        boolean goClockwise = clockwiseDistance < counterclockwiseDistance;

        int currentIndex = startVertexIndex;
        while (currentIndex != endVertexIndex) {
            Translation2d vertex = HEXAGON_VERTICES.get(currentIndex);
            detourWaypoints.add(new Waypoint(vertex, vertex, vertex));
            currentIndex = goClockwise
                ? (currentIndex + 1) % HEXAGON_VERTICES.size()
                : (currentIndex - 1 + HEXAGON_VERTICES.size()) % HEXAGON_VERTICES.size();
        }

        detourWaypoints.add(new Waypoint(HEXAGON_VERTICES.get(endVertexIndex), end, end));
        return detourWaypoints;
    }

    /**
     * Checks if a path between two points intersects any obstacle.
     *
     * @param start The starting point.
     * @param end   The target point.
     * @return      True if the path intersects an obstacle, false otherwise.
     */
    private boolean isPathIntersectingObstacle(Translation2d start, Translation2d end) {
        for (int i = 0; i < HEXAGON_VERTICES.size(); i++) {
            Translation2d vertex1 = HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = HEXAGON_VERTICES.get((i + 1) % HEXAGON_VERTICES.size());
            if (doLinesIntersect(start, end, vertex1, vertex2)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Finds the index of the closest hexagon vertex to a given point.
     *
     * @param point The point to check.
     * @return      The index of the closest vertex.
     */
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