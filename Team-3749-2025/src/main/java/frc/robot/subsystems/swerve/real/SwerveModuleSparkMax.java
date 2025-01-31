package frc.robot.subsystems.swerve.real;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
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

        private SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
        private SparkMaxConfig turnMotorConfig = new SparkMaxConfig();

        private EncoderConfig driveEncoderConfig = new EncoderConfig();
        private EncoderConfig turnEncoderConfig = new EncoderConfig();

        // private CANcoder absoluteEncoder;
        private double absoluteEncoderOffsetRad;

        private double driveAppliedVolts;
        private double turnAppliedVolts;

        /**
         * 
         * @param index
         * @param pidfConstants
         */
        public SwerveModuleSparkMax(int index) {

                driveEncoderConfig.positionConversionFactor((1 / ModuleConstants.driveMotorGearRatio) * Math.PI
                                * ModuleConstants.wheelDiameterMeters);
                driveEncoderConfig.velocityConversionFactor(
                                (1 / ModuleConstants.driveMotorGearRatio)
                                                * Units.rotationsPerMinuteToRadiansPerSecond(1)
                                                * (ModuleConstants.wheelDiameterMeters / 2.0));
                driveMotorConfig.encoder.apply(driveEncoderConfig);
                driveMotor = new SparkMax(DriveConstants.driveMotorPorts[index], SparkMax.MotorType.kBrushless);
                driveMotorConfig.inverted(DriveConstants.driveMotorReversed[index]);

                driveMotorConfig.smartCurrentLimit(MotorControllerConstants.standardStallLimit,
                                MotorControllerConstants.standardFreeLimit);
                driveMotorConfig.idleMode(IdleMode.kBrake);

                turnMotor = new SparkMax(DriveConstants.turningMotorPorts[index],
                                SparkMax.MotorType.kBrushless);

                turnMotorConfig.inverted(DriveConstants.turningMotorReversed[index]);
                turnEncoderConfig
                                .positionConversionFactor(1 / ModuleConstants.turnMotorGearRatio * (2 * Math.PI));
                turnEncoderConfig.velocityConversionFactor(
                                (1 / ModuleConstants.driveMotorGearRatio)
                                                * Units.rotationsPerMinuteToRadiansPerSecond(1));
                turnMotorConfig.encoder.apply(turnEncoderConfig);
                turnMotorConfig.smartCurrentLimit(MotorControllerConstants.relaxedStallLimit,
                                MotorControllerConstants.relaxedFreeLimit);
                turnMotorConfig.idleMode(IdleMode.kBrake);

                // absoluteEncoder = new CANcoder(DriveConstants.absoluteEncoderPorts[index]);
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
                        turnMotorConfig.closedLoop.pidf(ModuleConstants.turnPID[i][0], ModuleConstants.turnPID[i][1],
                                        ModuleConstants.turnPID[i][2],
                                        1 / 473,
                                        MotorControllerConstants.slots[i])
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, 2 * Math.PI);
                        turnMotorConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal,
                                        MotorControllerConstants.slots[i])
                                        .maxAcceleration(1, MotorControllerConstants.slots[i])
                                        .maxVelocity(1, MotorControllerConstants.slots[i])
                                        .allowedClosedLoopError(0.015, MotorControllerConstants.slots[i]);

                        turnMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                        // turnMotorConfig.closedLoop.
                }
                // turnMotorConfig

                // setpoint, control type, slot, FF from our code
                turningController.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);

                driveMotor.setControlFramePeriodMs(20);
                driveController = driveMotor.getClosedLoopController();

                for (int i = 0; i < 4; i++) {
                        driveMotorConfig.closedLoop.pidf(ModuleConstants.drivePID[i][0], ModuleConstants.drivePID[i][1],
                                        ModuleConstants.drivePID[i][2],
                                        1 / 473,
                                        MotorControllerConstants.slots[i]);
                        driveMotorConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal,
                                        MotorControllerConstants.slots[i])
                                        .maxAcceleration(DriveConstants.maxAccelerationMetersPerSecondSquared,
                                                        MotorControllerConstants.slots[i])
                                        .maxVelocity(DriveConstants.maxSpeedMetersPerSecond,
                                                        MotorControllerConstants.slots[i])
                                        .allowedClosedLoopError(DriveConstants.maxDriveVelocityError,
                                                        MotorControllerConstants.slots[i]);
                }

                driveController.setReference(1, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot2);

                driveMotor.configure(driveMotorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                turnMotor.configure(turnMotorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

        };

        @Override
        public void updateData(ModuleData data) {

                driveAppliedVolts = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();
                data.drivePositionM = getDrivePositionMeters();
                data.driveVelocityMPerSec = getDriveVelocityMetersPerSec();
                data.driveCurrentAmps = Math.abs(driveMotor.getOutputCurrent());
                data.driveTempCelcius = driveMotor.getMotorTemperature();

                turnAppliedVolts = turnMotor.getBusVoltage() * turnMotor.getAppliedOutput();
                data.turnAbsolutePositionRad = getAbsoluteTurningPositionRad();
                data.turnVelocityRadPerSec = turnMotor.getEncoder().getVelocity();
                // data.turnAbsolutePositionRad = getAbsoluteTurningPositionRad();
                // data.turnVelocityRadPerSec = getAbsoluteTurningVelocityRadPerSec();
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
                System.out.println(setpointPosition);
                if (Math.abs(setpointPosition - turnMotor.getEncoder().getPosition()) < 0.0174 * 8) {
                        System.out.println("0 PID");
                        turningController.setReference(setpointPosition, ControlType.kPosition,
                                        ClosedLoopSlot.kSlot2, feedforward);
                } else {
                        turningController.setReference(setpointPosition, ControlType.kPosition,
                                        ClosedLoopSlot.kSlot0, feedforward);
                }

        }



        private double getDrivePositionMeters() {
                return driveMotor.getEncoder().getPosition();
        };

        private double getAbsoluteTurningPositionRad() {
                double pos = Units.rotationsToRadians(turnMotor.getEncoder().getPosition())
                                - absoluteEncoderOffsetRad;
                while (pos < 0) {
                        pos += 2 * Math.PI;
                }
                while (pos > 2 * Math.PI) {
                        pos -= 2 * Math.PI;
                }
                return pos;
        };

        // private double getAbsoluteTurningVelocityRadPerSec() {
        // return
        // Units.rotationsToRadians(absoluteEncoder.getVelocity().getValueAsDouble());
        // };

        private double getDriveVelocityMetersPerSec() {

                return driveMotor.getEncoder().getVelocity();
        };

}