package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.swerve.GyroIO;

/**
 * Pigeon 2.0 implementation
 * 
 * @author Noah Simon
 */
public class PigeonGyro implements GyroIO {
    private final Pigeon2 pigeonGyro = new Pigeon2(30);
    private double yaw = 0;

    public PigeonGyro() {

        try {
            pigeonGyro.reset();
        } catch (Exception e) {
        }
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
                yaw+=360;
            }
            data.yawDeg = pigeonGyro.getYaw().getValueAsDouble() ;
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