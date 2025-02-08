package frc.robot.utils;

public class MotorData {
    public double appliedVolts;
    public double position;
    public double velocity;
    public double acceleration;

    public MotorData(double appliedVolts, double position, double velocity, double acceleration) {
        this.appliedVolts = appliedVolts;
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }
}