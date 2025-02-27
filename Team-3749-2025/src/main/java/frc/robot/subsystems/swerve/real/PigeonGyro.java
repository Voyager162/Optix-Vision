package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.swerve.GyroIO;

/**
 * Pigeon 2.0 implementation
 * 
 * @author Noah Simon
 */
public class PigeonGyro implements GyroIO {
    private final Pigeon2 pigeonGyro = new Pigeon2(30);
    private final Pigeon2Configuration config = new Pigeon2Configuration();
    private final GyroTrimConfigs trimConfig = new GyroTrimConfigs();
    private final MountPoseConfigs mountConfig = new MountPoseConfigs();
    private double yaw = 0;

    public PigeonGyro() {
        mountConfig.withMountPosePitch(0);
        mountConfig.withMountPoseYaw(90);
        mountConfig.withMountPoseRoll(-90);
        trimConfig.withGyroScalarX(-1.21);
        config.withGyroTrim(trimConfig);
        config.withMountPose(mountConfig);

        pigeonGyro.reset();
        pigeonGyro.getConfigurator().apply(config);

    }

    @Override
    public void updateData(GyroData data) {
        try {
            // +180 because it is mounted backwards
            yaw = pigeonGyro.getYaw().getValueAsDouble() + 180;
            while (yaw > 360) {
                yaw -= 360;
            }
            while (yaw < 0) {
                yaw += 360;
            }

            data.yawDeg = pigeonGyro.getYaw().getValueAsDouble();
            data.pitchDeg = pigeonGyro.getPitch().getValueAsDouble();
            data.rollDeg = pigeonGyro.getRoll().getValueAsDouble();
            data.isConnected = pigeonGyro.isConnected();

        } catch (Exception e) {
        }
    }

    @Override
    public void resetGyro() {
        pigeonGyro.reset();
    }

}