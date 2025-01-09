package frc.robot.subsystems.arm;

/**
 * arm interface holding data for the sim and real versions of the arm
 * @author Weston Gardner
 */
public interface ArmIO {

  public static class SubsystemData {
    public double positionUnits = 0;
    public double velocityUnits = 0;
    public double accelerationUnits = 0;
    public double inputVolts = 0;
    public double appliedVolts = 0;
    public double currentAmps = 0;
    public double tempCelcius = 0;
  }

  /* Updates the set of loggable inputs. */
  public default void updateData(SubsystemData data) {
  };

  /* Run the motor at the specified voltage. */
  public default void setVoltage(double volts) {
  };

  /* Enable or disable brake mode on the motor. */
  public default void setBrakeMode(boolean enable) {

  };

}
