package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.utils.MiscConstants.MotorControllerConstants;

/**
 * Sparkmax implementation for swerve mdoules
 * 
 * @author Noah Simon
 */

public class SwerveModuleSparkMax implements SwerveModuleIO {
        private SparkMax driveMotor;
        private SparkMax turnMotor;
        private SparkClosedLoopController turningController;
        private SparkClosedLoopController driveController;

        private CANcoder absoluteEncoder;
        private double absoluteEncoderOffsetRad;

        private double driveAppliedVolts;
        private double turnAppliedVolts;

        /**
         * 
         * @param index
         * @param pidfConstants
         */
        public SwerveModuleSparkMax(int index) {
                driveMotor = new SparkMax(DriveConstants.driveMotorPorts[index], SparkMax.MotorType.kBrushless);
                SparkMaxConfig driveConfig = new SparkMaxConfig();
                driveConfig.inverted(DriveConstants.driveMotorReversed[index]);
                driveConfig.encoder.positionConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * Math.PI
                                * ModuleConstants.wheelDiameterMeters);
                driveConfig.encoder.velocityConversionFactor(
                                (1 / ModuleConstants.driveMotorGearRatio)
                                                * Units.rotationsPerMinuteToRadiansPerSecond(1)
                                                * (ModuleConstants.wheelDiameterMeters / 2.0));
                driveConfig.smartCurrentLimit(DriveConstants.driveMotorStallLimit,
                                DriveConstants.driveMotorFreeLimit);
                driveConfig.idleMode(IdleMode.kBrake);
                driveMotor.configure(driveConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                turnMotor = new SparkMax(DriveConstants.turningMotorPorts[index],
                                SparkMax.MotorType.kBrushless);

                SparkMaxConfig turnConfig = new SparkMaxConfig();
                turnConfig.inverted(DriveConstants.turningMotorReversed[index]);
                turnConfig.encoder.positionConversionFactor(1 / ModuleConstants.turnMotorGearRatio * (2 * Math.PI));
                turnConfig.encoder.velocityConversionFactor(
                                (1 / ModuleConstants.driveMotorGearRatio)
                                                * Units.rotationsPerMinuteToRadiansPerSecond(1));
                turnConfig.smartCurrentLimit(DriveConstants.turnMotorStallLimit,
                                DriveConstants.turnMotorFreeLimit);
                turnConfig.idleMode(IdleMode.kBrake);

                turnMotor.configure(turnConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                absoluteEncoder = new CANcoder(DriveConstants.absoluteEncoderPorts[index]);
                absoluteEncoderOffsetRad = Units.degreesToRadians(DriveConstants.absoluteEncoderOffsetDeg[index]);

                driveMotor.setControlFramePeriodMs(20);

                // decrese when put in a seperate period. CHECK CAN BUS UTILIZATION
                turnMotor.setControlFramePeriodMs(20);

                // get the controller object
                turningController = turnMotor.getClosedLoopController();

                /**
                 * PID constants, and the motor velocity control
                 * FF is same for all of the same type of motor, and only for velocity
                 * different slots can hold different values for different purposes (velocity
                 * vs position, slow vs fast and large vs small)
                 */
                for (int i = 0; i < 4; i++) {
                        turnConfig.closedLoop.pidf(ModuleConstants.turnPID[i][0], ModuleConstants.turnPID[i][1], ModuleConstants.turnPID[i][2], 1 / 473,
                                        MotorControllerConstants.slots[i]);
                }

                // setpoint, control type, slot, FF from our code
                turningController.setReference(0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);

                driveMotor.setControlFramePeriodMs(20);
                driveController = driveMotor.getClosedLoopController();

                for (int i = 0; i < 4; i++) {
                        driveConfig.closedLoop.pidf(ModuleConstants.drivePID[i][0], ModuleConstants.drivePID[i][1], ModuleConstants.drivePID[i][2], 1 / 473,
                                        MotorControllerConstants.slots[i]);
                }

                driveController.setReference(0, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0);
                driveConfig.closedLoop.maxMotion
                                .maxVelocity(DriveConstants.maxSpeedMetersPerSecond)
                                .maxAcceleration(DriveConstants.maxAccelerationMetersPerSecondSquared)
                                .allowedClosedLoopError(DriveConstants.maxDriveVelocityError);

        };

        @Override
        public void updateData(ModuleData data) {

                driveAppliedVolts = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();
                data.drivePositionM = getDrivePositionMeters();
                data.driveVelocityMPerSec = getDriveVelocityMetersPerSec();
                data.driveAppliedVolts = driveAppliedVolts;
                data.driveCurrentAmps = Math.abs(driveMotor.getOutputCurrent());
                data.driveTempCelcius = driveMotor.getMotorTemperature();

                turnAppliedVolts = turnMotor.getBusVoltage() * turnMotor.getAppliedOutput();
                data.turnAbsolutePositionRad = getAbsoluteTurningPositionRad();
                data.turnVelocityRadPerSec = getAbsoluteTurningVelocityRadPerSec();
                data.turnAppliedVolts = turnAppliedVolts;
                data.turnCurrentAmps = Math.abs(turnMotor.getOutputCurrent());
                data.turnTempCelcius = turnMotor.getMotorTemperature();
        };

        @Override
        public void setDriveVelocityControl(double setpointVelocity, double feedforward) {
                if (Math.abs(setpointVelocity) < 0.25) {
                        driveController.setReference(setpointVelocity, ControlType.kMAXMotionVelocityControl,
                                        ClosedLoopSlot.kSlot2, feedforward);
                } else {
                        driveController.setReference(setpointVelocity, ControlType.kMAXMotionVelocityControl,
                                        ClosedLoopSlot.kSlot3, feedforward);
                }
        }

        @Override
        public void setTurningPositionControl(double setpointPosition, double feedforward) {
                if (Math.abs(setpointPosition) < 0.25) {
                        turningController.setReference(setpointPosition, ControlType.kMAXMotionVelocityControl,
                                        ClosedLoopSlot.kSlot0, feedforward);
                } else {
                        turningController.setReference(setpointPosition, ControlType.kMAXMotionVelocityControl,
                                        ClosedLoopSlot.kSlot1, feedforward);
                }
        }

        // public void set

        @Override
        public void setDriveVoltage(double volts) {
                driveAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                                DriveConstants.maxMotorVolts);
                driveAppliedVolts = MathUtil.applyDeadband(driveAppliedVolts, 0.05);
                driveMotor.setVoltage(driveAppliedVolts);
        };

        @Override
        public void setTurnVoltage(double volts) {
                turnAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                                DriveConstants.maxMotorVolts);
                turnAppliedVolts = MathUtil.applyDeadband(turnAppliedVolts, 0.05);

                turnMotor.setVoltage(turnAppliedVolts);
        };

        private double getDrivePositionMeters() {
                return driveMotor.getEncoder().getPosition();
        };

        private double getAbsoluteTurningPositionRad() {
                double pos = Units.rotationsToRadians(absoluteEncoder.getPosition().getValueAsDouble())
                                - absoluteEncoderOffsetRad;
                while (pos < 0) {
                        pos += 2 * Math.PI;
                }
                while (pos > 2 * Math.PI) {
                        pos -= 2 * Math.PI;
                }
                return pos;
        };

        private double getAbsoluteTurningVelocityRadPerSec() {
                return Units.rotationsToRadians(absoluteEncoder.getVelocity().getValueAsDouble());
        };

        private double getDriveVelocityMetersPerSec() {

                return driveMotor.getEncoder().getVelocity();
        };

}