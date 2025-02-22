package frc.robot.subsystems.arm.climb.real;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.arm.climb.ClimbArmIO;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.utils.OptixSpark;
import frc.robot.utils.UtilityFunctions;
import frc.robot.utils.MiscConstants.MotorControllerConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's sparkmax
 *
 * @author Elise Lin
 */
public class ClimbArmSparkMax implements ClimbArmIO {

	private OptixSpark frontMotor;
	private OptixSpark backMotor;

	private SparkAbsoluteEncoder absoluteEncoder;
	private double absolutePos;
	private double inputVolts = 0;
	private double previousVelocity = 0;
	private double velocity = 0;

	/**
	 * creates a new io implementation of a real arm that uses an absolute encoder
	 * with two motors
	 */
	public ClimbArmSparkMax() {

		System.out.println("[Init] Creating Climb Arm");

		frontMotor = new OptixSpark(ClimbArmConstants.frontMotorId, OptixSpark.Type.SPARKMAX);
		backMotor = new OptixSpark(ClimbArmConstants.backMotorId, OptixSpark.Type.SPARKMAX);

		frontMotor.setCurrentLimit(MotorControllerConstants.standardStallLimit,
				MotorControllerConstants.standardFreeLimit);
		frontMotor.setInverted(true);
		frontMotor.setBrakeMode(false);

		frontMotor.setVelocityConversionFactor((1 / ClimbArmConstants.armGearing) *
				(2 * Math.PI / 60.0));
		// frontMotor.setControlEncoder(FeedbackSensor.kAbsoluteEncoder);
		frontMotor.setPositionWrapping(0, 1);
		frontMotor.applyConfig();
		backMotor.applyConfig(frontMotor.getConfig());
		backMotor.setInverted(false);

		backMotor.applyConfig();

		absoluteEncoder = backMotor.getAbsoluteEncoder();

	}

	@Override
	public void setBrakeMode(boolean enabled) {
		frontMotor.setBrakeMode(enabled);
		backMotor.setBrakeMode(enabled);
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
		velocity = (frontMotor.getVelocity() + backMotor.getVelocity()) / 2;
		data.positionRad = getPosition();
		data.velocityRadPerSec = velocity;
		data.accelerationRadsPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
		data.frontMotorCurrentAmps = frontMotor.getCurrent();
		data.backMotorCurrentAmps = backMotor.getCurrent();
		data.inputVolts = inputVolts;
		data.frontMotorAppliedVolts = frontMotor.getAppliedVolts();
		data.backMotorAppliedVolts = backMotor.getAppliedVolts();
		data.frontMotorTempCelcius = frontMotor.getTemperature();
		data.backMotorTempCelcius = backMotor.getTemperature();
		SmartDashboard.putNumber("abs", absoluteEncoder.getPosition() * 2 * Math.PI);
	}

	/**
	 * sets the voltage of the motor
	 * 
	 * @param volts
	 */
	@Override
	public void setVoltage(double volts) {
		inputVolts = UtilityFunctions.applyDeadband(inputVolts, MotorControllerConstants.deadbandVoltage);
		inputVolts = MathUtil.clamp(volts, -12, 12);
		frontMotor.setVoltage(inputVolts);
		backMotor.setVoltage(inputVolts);
	}

	private double getPosition() {
		absolutePos = (absoluteEncoder.getPosition() * 2 * Math.PI) + Math.PI / 2;
		while (absolutePos > Math.PI * 2) {
			absolutePos -= Math.PI / 2;
		}
		return absolutePos;
	}
}