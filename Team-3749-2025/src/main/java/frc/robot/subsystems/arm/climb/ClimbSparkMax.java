package frc.robot.subsystems.arm.climb;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for the climb subsystem's sparkmax
 * I had to make two different motors because we are using two motors on the
 * climb
 * I have decided to have one of the motors be the primary motor taking care of
 * all the data
 * the secondary will just be there to provide more power to the climb
 * 
 * @author Elise Lin
 * @author Weston Gardner
 */
public class ClimbSparkMax implements ArmIO {

    private SparkMax leftMotor = new SparkMax(ClimbConstants.motorIds[0], MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(ClimbConstants.motorIds[1], MotorType.kBrushless);
    private SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    private EncoderConfig encoderConfig = new EncoderConfig();

    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    public ClimbSparkMax() {
        encoderConfig.positionConversionFactor(Math.PI * 2
                / ClimbConstants.armGearing);
        encoderConfig.velocityConversionFactor(Math.PI * 2
                / (60 * ClimbConstants.armGearing));

        leftMotorConfig.smartCurrentLimit(30, 50);
        leftMotorConfig.inverted(ClimbConstants.motorInverted[0]);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.encoder.apply(encoderConfig);

        rightMotorConfig.apply(leftMotorConfig);
        rightMotorConfig.inverted(ClimbConstants.motorInverted[1]);

        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * Takes in the data from ArmData and uses it to update the data on the
     * position of the motor, velocity, acceleration, applied volts on the motor,
     * and the motor temperature
     * 
     * @param data
     */
    @Override
    public void updateData(ArmData data) {
        previousVelocity = velocity;
        velocity = leftMotor.getEncoder().getVelocity();
        data.positionUnits = leftMotor.getEncoder().getPosition();
        data.velocityUnits = velocity;
        data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.currentAmps = leftMotor.getOutputCurrent();
        data.inputVolts = inputVolts;
        data.appliedVolts = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
        data.tempCelcius = leftMotor.getMotorTemperature();

    }

    /**
     * Takes the volts parameter, then uses inputVolts to set the motor voltage
     * Clamp takes the volts and makes sure that the amount of volts isn't above or
     * below the boundary for the voltage
     * Deadband makes it so that the value returns 0.0 if the volts is between -0.05
     * to 0.05
     * 
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.clamp(volts, -12.0, 12.0);
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        leftMotor.setVoltage(inputVolts);
        rightMotor.setVoltage(inputVolts);

    }
}
