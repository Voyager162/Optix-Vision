package frc.robot.subsystems.climb;

public interface ClimbIO {
    public static class ClimbData {
        public double positionRadians = 0;
        public double velocityRadiansPerSecond = 0;
        // public double accelerationRadiansPerSecondSquared = 0;
        public double currentAmps = 0;
        public double appliedVolts = 0;
        public double tempCelcius = 0;
        public boolean isClimbing = false;
    }

    public default void updateData(ClimbData data) {
    };

    public default void setVoltage(double volts) {
    };

    public default void setBrakeMode(boolean enable) {
    }

    public default void stop(){

    }

    public default void setIsClimbing(boolean isClimbing){

    }
}