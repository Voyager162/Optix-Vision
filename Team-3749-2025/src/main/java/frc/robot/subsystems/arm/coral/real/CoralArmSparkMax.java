package frc.robot.subsystems.arm.coral.real;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.arm.coral.CoralArmIO;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.utils.OptixSpark;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.MiscConstants.MotorControllerConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's sparkmax
 *
 * @author Elise Lin
 * @author Weston Gardner
 */
public class CoralArmSparkMax implements CoralArmIO {

	private OptixSpark motor;

	private SparkAbsoluteEncoder absoluteEncoder;
	private double absolutePos;

	private double previousVelocity = 0;

	/**
	 * creates an io implementation of a single real spark max motor
	 * 
	 * @param motorId
	 */
	public CoralArmSparkMax() {

		System.out.println("[Init] Creating Coral Arm");

		motor = new OptixSpark(CoralArmConstants.motorID, OptixSpark.Type.SPARKMAX);

		motor.setCurrentLimit(MotorControllerConstants.standardStallLimit, MotorControllerConstants.standardFreeLimit);
		motor.setInverted(true);
		motor.setBrakeMode(true);
		motor.setPositionConversionFactor(1 / CoralArmConstants.armGearing * 2 * Math.PI);
		motor.setVelocityConversionFactor(1 / CoralArmConstants.armGearing * 2 * Math.PI / 60.0);
		motor.setAbsoluteEncoderInverted(true);
		absoluteEncoder = motor.getAbsoluteEncoder();

		absolutePos = absoluteEncoder.getPosition();

		motor.setPosition(absolutePos);
		motor.setPositionWrapping(-Math.PI, Math.PI);
		motor.setControlEncoder(FeedbackSensor.kAbsoluteEncoder);
		motor.applyConfig();
	}

	@Override
	public void setBrakeMode(boolean enabled) {
		motor.setBrakeMode(enabled);
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
		double velocity = motor.getVelocity();
		data.positionRad = getPosition();
		data.velocityRadsPerSecond = velocity;
		data.accelerationRadsPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
		data.motorCurrentAmps = motor.getCurrent();
		data.motorAppliedVolts = motor.getAppliedVolts();
		data.motorTempCelcius = motor.getTemperature();
	}

	/**
	 * Sets the voltage of the motor
	 * 
	 * @param volts
	 */
	@Override
	public void setVoltage(double volts) {
		double inputVolts = UtilityFunctions.applyDeadband(volts, MotorControllerConstants.deadbandVoltage / 3.0);
		inputVolts = MathUtil.clamp(volts, -12, 12);
		// motor.setVoltage(inputVolts);
	}

	private double getPosition() {
		absolutePos = (absoluteEncoder.getPosition() * 2 * Math.PI) - CoralArmConstants.absoluteEncoderOffsetRad;
		while (absolutePos > Math.PI) {
			absolutePos -= Math.PI * 2;
		}
		while (absolutePos < -Math.PI) {
			absolutePos += Math.PI * 2;
		}
		return absolutePos;
	}

}
