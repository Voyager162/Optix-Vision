package frc.robot.subsystems.arm.real;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.GeneralArmConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's sparkmax
 *
 * @author Elise Lin
 */
public class ArmSparkMax implements ArmIO {

	private SparkMax firstMotor;
	private SparkMax secondMotor;
	private SparkMaxConfig firstMotorConfig = new SparkMaxConfig();
	private SparkMaxConfig secondMotorConfig = new SparkMaxConfig();

	private SparkAbsoluteEncoder absoluteEncoder;
	private double absolutePos;

	private double inputVolts = 0;
	private double previousVelocity = 0;
	private double velocity = 0;

	public ArmSparkMax(int firstMotorId, int secondMotorId) {

		firstMotor = new SparkMax(firstMotorId, MotorType.kBrushless);
		secondMotor = new SparkMax(secondMotorId, MotorType.kBrushless);

		firstMotorConfig.smartCurrentLimit(GeneralArmConstants.NEOStallLimit, GeneralArmConstants.NEOFreeLimit);
		firstMotorConfig.encoder.inverted(false);
		firstMotorConfig.inverted(false);
		firstMotorConfig.idleMode(IdleMode.kBrake);
		firstMotorConfig.encoder.positionConversionFactor(2 * Math.PI);
		firstMotorConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);

		firstMotor.configure(firstMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		secondMotorConfig.apply(firstMotorConfig);
		secondMotorConfig.encoder.inverted(true);
		secondMotorConfig.inverted(true);

		secondMotor.configure(secondMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		absoluteEncoder = firstMotor.getAbsoluteEncoder();
		absolutePos = absoluteEncoder.getPosition();

		firstMotor.getEncoder().setPosition(absolutePos);
		secondMotor.getEncoder().setPosition(absolutePos);
	}

	public void setIdleMode(IdleMode idleMode) {
		firstMotorConfig.idleMode(idleMode);
		secondMotorConfig.idleMode(idleMode);
		firstMotor.configure(firstMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		secondMotor.configure(secondMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	/**
	 * Takes in the data from ArmData and uses it to update the data on the position
	 * of the motor,
	 * velocity, acceleration, applied volts on the motor, and the motor temperature
	 *
	 * @param data
	 */
	@Override
	public void updateData(ArmData data) {
		previousVelocity = velocity;
		velocity = (firstMotor.getEncoder().getVelocity() + secondMotor.getEncoder().getVelocity()) / 2;
		// data.positionMeters = (leftMotor.getEncoder().getPosition() +
		// rightMotor.getEncoder().getVelocity()) / 2
		// + absolutePos;
		data.positionUnits = (firstMotor.getEncoder().getPosition() + secondMotor.getEncoder().getVelocity()) / 2;
		data.velocityUnits = velocity;
		data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
		data.firstMotorCurrentAmps = firstMotor.getOutputCurrent();
		data.secondMotorCurrentAmps = secondMotor.getOutputCurrent();
		data.inputVolts = inputVolts;
		data.firstMotorAppliedVolts = firstMotor.getBusVoltage() * firstMotor.getAppliedOutput();
		data.secondMotorAppliedVolts = secondMotor.getBusVoltage() * secondMotor.getAppliedOutput();
		data.firstMotorTempCelcius = firstMotor.getMotorTemperature();
		data.secondMotorTempCelcius = secondMotor.getMotorTemperature();
	}

	/**
	 * Takes the volts parameter, then uses inputVolts to set the motor voltage
	 * Clamp takes the volts
	 * and makes sure that the amount of volts isn't above or below the boundary for
	 * the voltage
	 * Deadband makes it so that the value returns 0.0 if the volts is between -0.05
	 * to 0.05
	 *
	 * @param volts
	 */
	@Override
	public void setVoltage(double volts) {
		inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
		inputVolts = MathUtil.clamp(volts, -12, 12);
		firstMotor.setVoltage(inputVolts);
		secondMotor.setVoltage(inputVolts);
	}
}
