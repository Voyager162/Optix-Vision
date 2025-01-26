package frc.robot.subsystems.elevator.real;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.EncoderConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.utils.MiscConstants;
import frc.robot.utils.MiscConstants.MotorControllerConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * SparkMax for elevator subsystem
 * 
 * @author Dhyan Soni
 */

public class ElevatorSparkMax implements ElevatorIO {
    private SparkMax leftMotor = new SparkMax(ElevatorConstants.ElevatorSpecs.motorIds[0], MotorType.kBrushless);
    private SparkMaxConfig leftMotorConfig = new SparkMaxConfig();

    private SparkMax rightMotor = new SparkMax(ElevatorConstants.ElevatorSpecs.motorIds[1], MotorType.kBrushless);
    private SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    private EncoderConfig encoderConfig = new EncoderConfig();

    private SparkAbsoluteEncoder absoluteEncoder;
    private double absolutePos;

    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    public ElevatorSparkMax() {
        System.out.println("[Init] Creating Elevator");

        encoderConfig.positionConversionFactor(Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / ElevatorConstants.ElevatorSpecs.gearing);
        encoderConfig.velocityConversionFactor(Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / (60 * ElevatorConstants.ElevatorSpecs.gearing));

        leftMotorConfig.smartCurrentLimit(MotorControllerConstants.standardStallLimit, MotorControllerConstants.standardFreeLimit);
        leftMotorConfig.inverted(ElevatorConstants.ElevatorSpecs.motorInverted[0]);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.encoder.apply(encoderConfig);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightMotorConfig.apply(leftMotorConfig);
        rightMotorConfig.inverted(ElevatorConstants.ElevatorSpecs.motorInverted[1]);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = leftMotor.getAbsoluteEncoder();
        absolutePos = absoluteEncoder.getPosition() + ElevatorConstants.ElevatorSpecs.zeroOffset;

        leftMotor.getEncoder().setPosition(absolutePos);
        rightMotor.getEncoder().setPosition(absolutePos);
    }

    public void setIdleMode(IdleMode idleMode) {
        leftMotorConfig.idleMode(idleMode);
        rightMotorConfig.idleMode(idleMode);
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateData(ElevatorData data) {
        previousVelocity = velocity;
        velocity = (leftMotor.getEncoder().getVelocity() + rightMotor.getEncoder().getVelocity()) / 2;
        // data.positionMeters = (leftMotor.getEncoder().getPosition() +
        // rightMotor.getEncoder().getVelocity()) / 2
        // + absolutePos;
        data.positionMeters = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) / 2;
        data.velocityMetersPerSecond = velocity;
        data.accelerationMetersPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.leftCurrentAmps = leftMotor.getOutputCurrent();
        data.rightCurrentAmps = rightMotor.getOutputCurrent();
        data.inputVolts = inputVolts;
        data.leftAppliedVolts = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
        data.rightAppliedVolts = rightMotor.getBusVoltage() * rightMotor.getAppliedOutput();
        data.leftTempCelcius = leftMotor.getMotorTemperature();
        data.rightTempCelcius = rightMotor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12, 12);
        leftMotor.setVoltage(inputVolts);
        rightMotor.setVoltage(inputVolts);
    }

}
