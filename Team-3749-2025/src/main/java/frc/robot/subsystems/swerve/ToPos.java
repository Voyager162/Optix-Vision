package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ToPos {
    private static final ExecutorService executorService = Executors.newSingleThreadExecutor();

    // Hexagon vertices defined as obstacles (updated with loop closure)
    private static final List<Translation2d> HEXAGON_VERTICES = List.of(
            new Translation2d(3.76, 3.49),
            new Translation2d(3.66, 4.39),
            new Translation2d(4.39, 4.93),
            new Translation2d(5.22, 4.57),
            new Translation2d(5.32, 3.67),
            new Translation2d(4.59, 3.13),
            new Translation2d(3.76, 3.49) // Closing the loop
    );

    // Robot dimensions (width and height in meters)
    private static final double ROBOT_WIDTH = 0.6;
    private static final double ROBOT_HEIGHT = 0.6;

    /**
     * Asynchronously generates a dynamic path using PathPlanner.
     *
     * @param initialPose            Starting pose
     * @param finalPose              Target pose
     * @param maxVelocity            Maximum velocity constraint
     * @param maxAcceleration        Maximum acceleration constraint
     * @param maxAngularVelocity     Maximum angular velocity constraint
     * @param maxAngularAcceleration Maximum angular acceleration constraint
     * @param initialSpeed           Initial speed of the robot
     * @param initialHeading         Initial heading of the robot
     */
    public static void generateDynamicPathAsync(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration,
            double initialSpeed,
            double initialHeading) {
        executorService.submit(() -> {
            System.out.println("Starting dynamic path generation...");
            PathPlannerPath path = generateDynamicPath(
                    initialPose, finalPose, maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration, initialSpeed, initialHeading);
            if (path != null) {
                System.out.println("Dynamic path generated successfully.");
            } else {
                System.out.println("Dynamic path generation failed.");
            }
        });
    }

    /**
     * Generates a dynamic path avoiding obstacles using PathPlanner.
     *
     * @param initialPose            Starting pose
     * @param finalPose              Target pose
     * @param maxVelocity            Maximum velocity constraint
     * @param maxAcceleration        Maximum acceleration constraint
     * @param maxAngularVelocity     Maximum angular velocity constraint
     * @param maxAngularAcceleration Maximum angular acceleration constraint
     * @param initialSpeed           Initial speed of the robot
     * @param initialHeading         Initial heading of the robot
     * @return A PathPlannerPath if successful, null otherwise
     */
    public static PathPlannerPath generateDynamicPath(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration,
            double initialSpeed,
            double initialHeading) {
        List<Waypoint> waypoints = new ArrayList<>();

        // Define initial waypoint
        Translation2d start = initialPose.getTranslation();
        Translation2d end = finalPose.getTranslation();
        waypoints.add(new Waypoint(start, start, start.plus(new Translation2d(0.5, 0.0))));

        // Check for obstacles and calculate detours
        if (!isPathClear(start, end)) {
            System.out.println("Obstacle detected. Calculating detour...");
            List<Translation2d> detourPoints = calculateDetours(start, end);
            for (Translation2d detour : detourPoints) {
                System.out.println("Adding detour waypoint at: " + detour);
                waypoints.add(new Waypoint(detour, detour, detour));
            }
        } else {
            System.out.println("No obstacles detected. Direct path is clear.");
        }

        // Add a waypoint 1 meter before the final pose for heading alignment
        Translation2d approachPoint = end.minus(new Translation2d(
                1.0 * Math.cos(finalPose.getRotation().getRadians()),
                1.0 * Math.sin(finalPose.getRotation().getRadians())));
        waypoints.add(new Waypoint(approachPoint, approachPoint, approachPoint));

        // Add final waypoint
        waypoints.add(new Waypoint(end, end, end.minus(new Translation2d(0.5, 0.0))));

        if (waypoints.size() < 2) {
            System.out.println("Error: Not enough waypoints for path generation.");
            return null;
        }

        // Set path constraints
        PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);

        try {
            GoalEndState goalEndState = new GoalEndState(0.0, finalPose.getRotation());
            PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, goalEndState);
            path.preventFlipping = true;
            return path;
        } catch (Exception e) {
            System.out.println("PathPlanner Error: " + e.getMessage());
            return null;
        }
    }

    /**
     * Checks if a direct path is clear of obstacles, accounting for robot dimensions.
     *
     * @param start Starting point
     * @param end   Ending point
     * @return True if the path is clear, false otherwise
     */
    private static boolean isPathClear(Translation2d start, Translation2d end) {
        for (int i = 0; i < HEXAGON_VERTICES.size() - 1; i++) {
            Translation2d vertex1 = HEXAGON_VERTICES.get(i);
            Translation2d vertex2 = HEXAGON_VERTICES.get(i + 1);
            if (linesIntersectWithRobotBounds(start, end, vertex1, vertex2)) {
                System.out.println("Path intersects obstacle edge: " + vertex1 + " to " + vertex2);
                return false;
            }
        }
        return true;
    }

    /**
     * Determines if two line segments intersect, considering robot dimensions.
     *
     * @param p1 Start of the first segment
     * @param p2 End of the first segment
     * @param q1 Start of the second segment
     * @param q2 End of the second segment
     * @return True if the segments intersect, false otherwise
     */
    private static boolean linesIntersectWithRobotBounds(Translation2d p1, Translation2d p2, Translation2d q1, Translation2d q2) {
        double halfDiagonal = Math.sqrt(Math.pow(ROBOT_WIDTH / 2, 2) + Math.pow(ROBOT_HEIGHT / 2, 2));

        Translation2d offset1 = new Translation2d(-halfDiagonal, -halfDiagonal);
        Translation2d offset2 = new Translation2d(halfDiagonal, halfDiagonal);

        Translation2d adjustedP1 = p1.plus(offset1);
        Translation2d adjustedP2 = p2.plus(offset2);

        double det = (adjustedP2.getX() - adjustedP1.getX()) * (q2.getY() - q1.getY()) - (adjustedP2.getY() - adjustedP1.getY()) * (q2.getX() - q1.getX());
        if (det == 0) {
            return false;
        }

        double lambda = ((q2.getY() - q1.getY()) * (q2.getX() - adjustedP1.getX()) - (q2.getX() - q1.getX()) * (q2.getY() - adjustedP1.getY())) / det;
        double gamma = ((adjustedP1.getY() - adjustedP2.getY()) * (q2.getX() - adjustedP1.getX()) + (adjustedP2.getX() - adjustedP1.getX()) * (q2.getY() - adjustedP1.getY())) / det;

        return (0 <= lambda && lambda <= 1) && (0 <= gamma && gamma <= 1);
    }

    /**
     * Calculates detour points to avoid obstacles between start and end.
     *
     * @param start Starting point
     * @param end   Ending point
     * @return List of detour points
     */
    private static List<Translation2d> calculateDetours(Translation2d start, Translation2d end) {
        List<Translation2d> detours = new ArrayList<>();

        // Example detour logic: Offset path to the left or right
        Translation2d midpoint = start.interpolate(end, 0.5);
        Translation2d offset = new Translation2d(-(end.getY() - start.getY()), end.getX() - start.getX()).div(midpoint.getNorm()).times(0.5);

        Translation2d detour1 = midpoint.plus(offset);
        Translation2d detour2 = midpoint.minus(offset);

        if (isPathClear(start, detour1) && isPathClear(detour1, end)) {
            detours.add(detour1);
        } else if (isPathClear(start, detour2) && isPathClear(detour2, end)) {
            detours.add(detour2);
        }

        return detours;
    }

    /**
     * Stops the thread for path generation when no longer needed.
     */
    public static void shutdown() {
        executorService.shutdownNow();
        System.out.println("Path generation service stopped.");
    }
}