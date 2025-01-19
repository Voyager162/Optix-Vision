package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.real.VisionReal;
import frc.robot.subsystems.vision.VisionIO.*;
import frc.robot.utils.ShuffleData;

public class Vision extends SubsystemBase {
    private ShuffleData<Double> visionLogX = new ShuffleData<Double>("Vision", "Pose: X", -1.0);
    private ShuffleData<Double> visionLogY = new ShuffleData<Double>("Vision", "Pose: Y", -1.0);
    private ShuffleData<Double> visionLogRotation = new ShuffleData<Double>("Vision", "Pose: Rotation", -1.0);

    private ShuffleData<String> visionLogLatency = new ShuffleData<String>("Vision", "Latency", "-1.0");
    private ShuffleData<String> visionLogTargetsSeen = new ShuffleData<String>("Vision", "Targets Seen", "-1.0");

    VisionIO visionIO;
    VisionData visionData = new VisionData();

    Vision() {
        if (Robot.isReal()) {
            visionIO = new VisionReal(visionData);
        } else {
            throw new Error("Vision simulation not implemented");
        }
    }

    @Override
    public void periodic() {
        visionIO.updatePose();

        visionLogX.set(visionData.visionEstimatedPose.getTranslation().getX());
        visionLogY.set(visionData.visionEstimatedPose.getTranslation().getY());
        visionLogRotation.set(visionData.visionEstimatedPose.getRotation().getRadians());


        //? Did this weird string thing because the number of cameras we use can change so can't hardcode each log & im too lazy
        String latencyStr = "[";
        for (double latency : visionData.latencyMillis) {
            latencyStr += latency + ", ";
        }
        latencyStr += "]";
        visionLogLatency.set(latencyStr);

        String targetsSeenStr = "[";
        for (double targetsSeen : visionData.targetsSeen) {
            targetsSeenStr += targetsSeen + ", ";
        }
        targetsSeenStr += "]";
        visionLogTargetsSeen.set(targetsSeenStr);
    }
}
