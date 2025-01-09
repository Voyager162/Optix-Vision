package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public static class ElevatorData {
        double positionRad = 0;
        double positionReal = 0;
    }

    public default void updateData(ElevatorData data){
    };

    public default void setVoltage(double volts){
    };
}

