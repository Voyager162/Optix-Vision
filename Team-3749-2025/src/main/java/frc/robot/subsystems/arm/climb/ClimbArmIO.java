package frc.robot.subsystems.arm.climb;

/**
 * Interface file for arm subsystem
 *
 * @author Weston Gardner
 */
public interface ClimbArmIO {

	public static class ArmData {
		public double appliedVolts = 0;
		public double positionRad = 0;
		public double velocityRadPerSec = 0;
		public double accelerationUnits = 0;
		public double inputVolts = 0;
		public double frontMotorCurrentAmps = 0;
		public double backMotorCurrentAmps = 0;
		public double frontMotorAppliedVolts = 0;
		public double backMotorAppliedVolts = 0;
		public double frontMotorTempCelcius = 0;
		public double backMotorTempCelcius = 0;
	}

	/**
	 * Updates the set of loggable inputs.
	 * 
	 * @param data
	 */
	public default void updateData(ArmData data) {
	};

	/**
	 * Run the motor at the specified voltage.
	 * 
	 * @param volts
	 */
	public default void setVoltage(double volts) {
	};

	/**
	 * Enable or disable brake mode on the motor.
	 * 
	 * @param enable
	 */
	public default void setBrakeMode(boolean enable) {
	};

	public default void setPosition(double setpointPositionRad, double feedforward) {
	}

}
