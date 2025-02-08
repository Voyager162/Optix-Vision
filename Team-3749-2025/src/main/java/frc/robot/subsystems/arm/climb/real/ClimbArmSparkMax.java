package frc.robot.subsystems.arm.climb.real;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.arm.climb.ClimbArmIO;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.utils.OptixSpark;
import frc.robot.utils.MiscConstants.MotorControllerConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's sparkmax
 *
 * @author Elise Lin
 */
public class ClimbArmSparkMax implements ClimbArmIO {

	private OptixSpark frontMotorLead;
	private OptixSpark backMotorFollower;

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
		frontMotorLead = new OptixSpark(ClimbArmConstants.frontMotorId, OptixSpark.Type.SPARKMAX);
		backMotorFollower = new OptixSpark(ClimbArmConstants.backMotorId, OptixSpark.Type.SPARKMAX);

		frontMotorLead.setCurrentLimit(MotorControllerConstants.standardStallLimit, MotorControllerConstants.standardFreeLimit);
		frontMotorLead.setInverted(false);
		frontMotorLead.setBrakeMode(true);
		frontMotorLead.setPositionConversionFactor(1 / ClimbArmConstants.armGearing * 2 * Math.PI);
		frontMotorLead.setVelocityConversionFactor(1 / ClimbArmConstants.armGearing * 2 * Math.PI / 60.0);
		frontMotorLead.setPID(ClimbArmConstants.kP, ClimbArmConstants.kI, ClimbArmConstants.kD, ClosedLoopSlot.kSlot0);

		backMotorFollower.applyConfig(frontMotorLead.getConfig());
		backMotorFollower.setInverted(true);

		absoluteEncoder = frontMotorLead.getAbsoluteEncoder();
		absolutePos = absoluteEncoder.getPosition();

		frontMotorLead.setPosition(absolutePos);
		backMotorFollower.setPosition(absolutePos);
		backMotorFollower.setFollow(frontMotorLead);


	}

	@Override
	public void setBrakeMode(boolean enabled) {
		frontMotorLead.setBrakeMode(enabled);
		backMotorFollower.setBrakeMode(enabled);
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
		velocity = (frontMotorLead.getVelocity() + backMotorFollower.getVelocity()) / 2;
		data.positionUnits = (frontMotorLead.getPosition() + backMotorFollower.getVelocity()) / 2;
		data.velocityUnits = velocity;
		data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
		data.frontMotorCurrentAmps = frontMotorLead.getCurrent();
		data.backMotorCurrentAmps = backMotorFollower.getCurrent();
		data.inputVolts = inputVolts;
		data.frontMotorAppliedVolts = frontMotorLead.getAppliedVolts();
		data.backMotorAppliedVolts = backMotorFollower.getAppliedVolts();
		data.frontMotorTempCelcius = frontMotorLead.getTemperature();
		data.backMotorTempCelcius = backMotorFollower.getTemperature();
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
		frontMotorLead.setVoltage(inputVolts);
	}

	@Override
	public void setPosition(double setpointPositionRad, double feedforward) {
		frontMotorLead.setPositionControl(setpointPositionRad, feedforward);
	}
}
