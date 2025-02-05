package frc.robot.subsystems.swerve.real;

import frc.robot.utils.OptixSpark;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.utils.MiscConstants.MotorControllerConstants;

public class SwerveModuleSpark implements SwerveModuleIO {

    private OptixSpark drive;
    private OptixSpark turn;

    private CANcoder absoluteEncoder;

    private double absoluteEncoderOffsetRad;

    public SwerveModuleSpark(int index) {
        drive = new OptixSpark(DriveConstants.driveMotorIds[index], OptixSpark.Type.SPARKFLEX);
        drive.setPositionConversionFactor(
                1 / ModuleConstants.driveMotorGearRatio * Math.PI * ModuleConstants.wheelDiameterMeters);
        drive.setVelocityConversionFactor(
                1 / ModuleConstants.driveMotorGearRatio * Math.PI * ModuleConstants.wheelDiameterMeters * (1 / 60.0));
        drive.setInverted(DriveConstants.driveMotorInverted[index]);
        drive.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        drive.setBrakeMode(true);

        turn = new OptixSpark(DriveConstants.turnMotorIds[index], OptixSpark.Type.SPARKMAX);
        turn.setPositionConversionFactor(1 / ModuleConstants.turnMotorGearRatio * Math.PI * 2);
        turn.setVelocityConversionFactor(1 / ModuleConstants.turnMotorGearRatio * Math.PI * 2 * (1 / 60.0));
        turn.setInverted(DriveConstants.turningMotorInverted[index]);
        turn.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        turn.setBrakeMode(true);
        turn.setPositionWrapping(0, 2 * Math.PI);

        for (int i = 0; i < 4; i++) {
            turn.setPIDF(
                    ModuleConstants.turnPID[i][0],
                    ModuleConstants.turnPID[i][1],
                    ModuleConstants.turnPID[i][2],
                    0,
                    MotorControllerConstants.slots[i]);

            drive.setPIDF(
                    ModuleConstants.drivePID[i][0],
                    ModuleConstants.drivePID[i][1],
                    ModuleConstants.drivePID[i][2],
                    0,
                    MotorControllerConstants.slots[i]);
        }

        absoluteEncoder = new CANcoder(DriveConstants.absoluteEncoderIds[index]);
        absoluteEncoderOffsetRad = Units.degreesToRadians(DriveConstants.absoluteEncoderOffsetDeg[index]);

    }

    @Override
    public void updateData(ModuleData data) {
        data.driveAppliedVolts = drive.getAppliedVolts();
        data.drivePositionM = drive.getPosition();
        data.driveVelocityMPerSec = drive.getVelocity();
        data.driveCurrentAmps = drive.getCurrent();
        data.driveTempCelcius = drive.getTemperature();

        data.turnAppliedVolts = turn.getAppliedVolts();
        data.turnPositionRad = turn.getPosition();
        data.turnVelocityRadPerSec = turn.getVelocity();
        data.turnCurrentAmps = turn.getCurrent();
        data.turnTempCelcius = turn.getTemperature();

    };

    @Override
    public void setDriveVelocity(double setpointVelocity, double feedforward) {
        drive.setVelocity(setpointVelocity, feedforward);
    }

    @Override
    public void setTurnPosition(double setpointVelocity, double feedforward) {
        drive.setVelocity(setpointVelocity, feedforward);
    }

    /** Run the drive motor at the specified voltage. */
    @Override
    public void setDriveVoltage(double volts) {
        drive.setVoltage(volts);
    }

    /** Run the turn motor at the specified voltage. */
    @Override
    public void setTurnVoltage(double volts) {
        turn.setVoltage(volts);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveBrakeMode(boolean enable) {
        drive.setBrakeMode(enable);
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setTurningBrakeMode(boolean enable) {
        turn.setBrakeMode(enable);
    }

}
