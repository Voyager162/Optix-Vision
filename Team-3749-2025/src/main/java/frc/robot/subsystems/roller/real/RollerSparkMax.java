package frc.robot.subsystems.roller.real;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.utils.MiscConstants.MotorControllerConstants;

public class RollerSparkMax implements RollerIO {
    private SparkMax rollerMotor;
    private SparkMaxConfig motorConfig = new SparkMaxConfig();
    private EncoderConfig encoderConfig = new EncoderConfig();

    public RollerSparkMax(int motorID, double positionConversionFactor, boolean isInverted) {
        rollerMotor = new SparkMax(motorID, MotorType.kBrushless);
        motorConfig.smartCurrentLimit(MotorControllerConstants.relaxedStallLimit,
                MotorControllerConstants.relaxedFreeLimit);
        motorConfig.inverted(isInverted);
        motorConfig.idleMode(IdleMode.kBrake);
        encoderConfig.positionConversionFactor(positionConversionFactor);
        encoderConfig.velocityConversionFactor(positionConversionFactor / 60.0);
        motorConfig.encoder.apply(encoderConfig);
        rollerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateData(RollerData data) {
        data.rollerAppliedVolts = rollerMotor.getBusVoltage() * rollerMotor.getAppliedOutput();
        data.rollerVelocityRadPerSec = rollerMotor.getEncoder().getVelocity();
        data.rollerTempCelcius = rollerMotor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        rollerMotor.setVoltage(volts);
    }
}
