package frc.robot.subsystems.arm.climb.real;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		frontMotor = new OptixSpark(ClimbArmConstants.frontMotorId, OptixSpark.Type.SPARKMAX);
		backMotor = new OptixSpark(ClimbArmConstants.backMotorId, OptixSpark.Type.SPARKMAX);

		frontMotor.setCurrentLimit(MotorControllerConstants.standardStallLimit,
				MotorControllerConstants.standardFreeLimit);
		frontMotor.setInverted(true);
		frontMotor.setBrakeMode(false);
		frontMotor.setVelocityConversionFactor(1 / ClimbArmConstants.armGearing *
				2 * Math.PI / 60.0);
		frontMotor.setPID(ClimbArmConstants.kP.get(), ClimbArmConstants.kI.get(), ClimbArmConstants.kD.get(),
				ClosedLoopSlot.kSlot0);

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
		data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
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
		inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
		inputVolts = MathUtil.clamp(volts, -12, 12);
		frontMotor.setVoltage(inputVolts);
		backMotor.setVoltage(inputVolts);
	}

	@Override
	public void setPosition(double setpointPositionRad, double feedforward) {
		frontMotor.setPositionControl(setpointPositionRad, feedforward);
		backMotor.setPositionControl(setpointPositionRad, feedforward);
	}

	private double getPosition() {
		absolutePos = absoluteEncoder.getPosition() * 2 * Math.PI - ClimbArmConstants.absoluteEncoderOffsetRad;
		while (absolutePos < 0) {
			absolutePos += 2 * Math.PI;
		}
		while (absolutePos > 2 * Math.PI) {
			absolutePos -= 2 * Math.PI;
		}
		return absolutePos;
	}
}