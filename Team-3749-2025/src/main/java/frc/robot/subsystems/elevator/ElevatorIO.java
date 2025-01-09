package frc.robot.subsystems.elevator;

/**
 * IO implementation for elevator subsystem
 * 
 * @author Dhyan Soni
 */

public interface ElevatorIO {
    public static class ElevatorData {
        public double positionUnits = 0;
        public double velocityUnits = 0;
        public double accelerationUnits = 0;
        public double inputVolts = 0;
        public double appliedVolts = 0;
        public double currentAmps = 0;
        public double tempCelcius = 0;
    }

    public default void updateData(ElevatorData data){
    };

    public default void setVoltage(double volts){
    };
}

