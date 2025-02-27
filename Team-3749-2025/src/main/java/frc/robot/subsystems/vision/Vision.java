package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.*;
import frc.robot.subsystems.vision.real.Photonvision;

public class Vision extends SubsystemBase {
    @SuppressWarnings("unused")
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
    }
}
