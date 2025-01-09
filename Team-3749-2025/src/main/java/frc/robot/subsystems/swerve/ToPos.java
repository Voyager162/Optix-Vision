package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;

public class ToPos {

    // Instance of the LocalADStar pathfinder
    private static final LocalADStar pathfinder = new LocalADStar();

    // Static method to generate a dynamic path while accounting for predefined obstacles
    public static PathPlannerPath generateDynamicPath(
            Pose2d initialPose,
            Pose2d finalPose,
            double maxVelocity,
            double maxAcceleration,
            double maxAngularVelocity,
            double maxAngularAcceleration
    ) {
        // Define path constraints
        PathConstraints constraints = new PathConstraints(
                maxVelocity,
                maxAcceleration,
                maxAngularVelocity,
                maxAngularAcceleration
        );

        // Set the start and goal positions for the pathfinder
        pathfinder.setStartPosition(initialPose.getTranslation());
        pathfinder.setGoalPosition(finalPose.getTranslation());

        // Predefine static obstacles for the Blue Alliance reef section
        List<Pair<Translation2d, Translation2d>> reefBoundingBoxes = getBlueAllianceReefBoundingBoxes();

        // Set dynamic obstacles
        pathfinder.setDynamicObstacles(reefBoundingBoxes, initialPose.getTranslation());

        // Wait for the pathfinder to calculate a new path with a timeout

        // Retrieve the calculated path
        PathPlannerPath path = pathfinder.getCurrentPath(
                constraints,
                new GoalEndState(0.0, finalPose.getRotation()) // Goal end state with final heading
        );

        return path;
    }

    // Method to define static obstacles for the Blue Alliance reef section as bounding boxes
    private static List<Pair<Translation2d, Translation2d>> getBlueAllianceReefBoundingBoxes() {
        List<Pair<Translation2d, Translation2d>> boundingBoxes = new ArrayList<>();

        // Define the reef as a 4x4 square centered at (4,4)
        Translation2d bottomLeft = new Translation2d(2.0, 2.0); // Bottom-left corner of the reef
        Translation2d topRight = new Translation2d(6.0, 6.0);   // Top-right corner of the reef

        // Add the reef bounding box
        boundingBoxes.add(new Pair<>(bottomLeft, topRight));

        return boundingBoxes;
    }
}
