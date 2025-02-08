package frc.robot.subsystems.arm.coral;

/**
 * Interface file for arm subsystem
 *
 * @author Weston Gardner
 */
public interface CoralArmIO {

	public static class ArmData {
		public double positionUnits = 0;
		public double velocityUnits = 0;
		public double accelerationUnits = 0;
		public double motorCurrentAmps = 0;
		public double motorAppliedVolts = 0;
		public double motorTempCelcius = 0;
        public Double inputVolts;
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

	public default void setPosition(double setpointPositionRad, double feedforward) {
	}

	/**
	 * Enable or disable brake mode on the motor.
	 * 
	 * @param enable
	 */
	public default void setBrakeMode(boolean enable) {
	};

}
