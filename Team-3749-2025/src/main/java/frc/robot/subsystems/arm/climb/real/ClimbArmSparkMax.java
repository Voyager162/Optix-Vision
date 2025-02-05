package frc.robot.subsystems.arm.climb.real;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.climb.ClimbArmIO;
import frc.robot.subsystems.arm.climb.ClimbConstants;
import frc.robot.utils.OptixSpark;
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
	 * 
	 * @param frontMotorId
	 * @param backMotorId
	 */
	public ClimbArmSparkMax() {
		frontMotor = new OptixSpark(ClimbConstants.frontMotorId, OptixSpark.Type.SPARKMAX);
		backMotor = new OptixSpark(ClimbConstants.backMotorId, OptixSpark.Type.SPARKMAX);

		frontMotor.setCurrentLimit(ArmConstants.NEOStallLimit, ArmConstants.NEOFreeLimit);
		frontMotor.setInverted(false);
		frontMotor.setBrakeMode(true);
		frontMotor.setPositionConversionFactor(1 / ClimbConstants.armGearing * 2 * Math.PI);
		frontMotor.setVelocityConversionFactor(1 / ClimbConstants.armGearing * 2 * Math.PI / 60.0);


		backMotor.applyConfig(frontMotor.getConfig());
		backMotor.setInverted(true);


		absoluteEncoder = frontMotor.getAbsoluteEncoder();
		absolutePos = absoluteEncoder.getPosition();

		frontMotor.setPosition(absolutePos);
		backMotor.setPosition(absolutePos);
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
		data.positionUnits = (frontMotor.getPosition() + backMotor.getVelocity()) / 2;
		data.velocityUnits = velocity;
		data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
		data.frontMotorCurrentAmps = frontMotor.getCurrent();
		data.backMotorCurrentAmps = backMotor.getCurrent();
		data.inputVolts = inputVolts;
		data.frontMotorAppliedVolts = frontMotor.getAppliedVolts();
		data.backMotorAppliedVolts = backMotor.getAppliedVolts();
		data.frontMotorTempCelcius = frontMotor.getTemperature();
		data.backMotorTempCelcius = backMotor.getTemperature();
	}
 
	/**
	 * sets the voltage of the motor
	 * 
	 * @param volts
	 */
	@Override
	public void setVoltage(double volts) {
		inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
		inputVolts = MathUtil.clamp(volts, -12, 12);
		frontMotor.setVoltage(inputVolts);
		backMotor.setVoltage(inputVolts);
	}
}
