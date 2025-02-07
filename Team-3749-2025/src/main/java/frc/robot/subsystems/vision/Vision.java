package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.real.Photonvision;
import frc.robot.subsystems.vision.VisionIO.*;

public class Vision extends SubsystemBase {
    private VisionIO visionIO;
    private VisionData visionData = new VisionData();

    public Vision() {
        if (Robot.isReal()) {
            visionIO = new Photonvision(visionData);
        } else {
            throw new Error("Vision simulation not implemented");
        }
    }

    @Override
    public void periodic() {
        visionIO.updatePose();

        for (int i = 0; i < visionData.visionEstimatedPoses.length; i++) {
            SmartDashboard.putNumber("Vision/Cam" + i + "/latency", visionData.latencyMillis[i]);
            SmartDashboard.putNumber("Vision/Cam" + i + "/targetsSeen", visionData.targetsSeen[i]);

            double x = (visionData.visionEstimatedPoses[i].getX());
            double y = (visionData.visionEstimatedPoses[i].getY());
            double rotation = (visionData.visionEstimatedPoses[i].getRotation().getDegrees());

            SmartDashboard.putNumberArray("Vision/Cam" + i + "/pose", new double[] { x, y, rotation });
        }
    }
}
