package frc.robot.subsystems.swerve.real;

import frc.robot.utils.OptixSpark;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveConstants.ControlConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveConstants.MotorConstants;
import frc.robot.utils.MiscConstants.MotorControllerConstants;

public class SwerveModuleSpark implements SwerveModuleIO {

    private OptixSpark drive;
    private OptixSpark turn;

    private CANcoder absoluteEncoder;

    private double absoluteEncoderOffsetRad;

    public SwerveModuleSpark(int index) {
        drive = new OptixSpark(MotorConstants.driveMotorIds[index], OptixSpark.Type.SPARKFLEX);
        drive.setPositionConversionFactor(
                1 / MotorConstants.driveMotorGearRatio * Math.PI * DrivetrainConstants.wheelDiameterMeters);
        drive.setVelocityConversionFactor(
                1 / MotorConstants.driveMotorGearRatio * Math.PI * DrivetrainConstants.wheelDiameterMeters
                        * (1 / 60.0));
        drive.setInverted(MotorConstants.driveMotorInverted[index]);
        drive.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        drive.setBrakeMode(true);

        turn = new OptixSpark(MotorConstants.turnMotorIds[index], OptixSpark.Type.SPARKMAX);
        turn.setPositionConversionFactor(1 / MotorConstants.turnMotorGearRatio * Math.PI * 2);
        turn.setVelocityConversionFactor(1 / MotorConstants.turnMotorGearRatio * Math.PI * 2 * (1 / 60.0));
        turn.setInverted(MotorConstants.turningMotorInverted[index]);
        turn.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        turn.setBrakeMode(true);
        turn.setPositionWrapping(0, 2 * Math.PI);

        for (int i = 0; i < 4; i++) {
            turn.setPID(
                    ControlConstants.turnPID[i][0],
                    ControlConstants.turnPID[i][1],
                    ControlConstants.turnPID[i][2],

                    MotorControllerConstants.slots[i]);

            drive.setPID(
                    ControlConstants.drivePID[i][0],
                    ControlConstants.drivePID[i][1],
                    ControlConstants.drivePID[i][2],

                    MotorControllerConstants.slots[i]);
        }

        absoluteEncoder = new CANcoder(MotorConstants.absoluteEncoderIds[index]);
        absoluteEncoderOffsetRad = Units.degreesToRadians(MotorConstants.absoluteEncoderOffsetDeg[index]);
        turn.setPosition(absoluteEncoder.getPosition().getValueAsDouble() * 2 * Math.PI - absoluteEncoderOffsetRad);
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
        drive.setVelocityControl(setpointVelocity, feedforward);
    }

    @Override
    public void setTurnPosition(double setpointVelocity, double feedforward) {
        drive.setVelocityControl(setpointVelocity, feedforward);
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
