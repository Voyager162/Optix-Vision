package frc.robot.utils;

import java.util.Map;
import java.util.function.Consumer;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;;

/**
 * SysId assisted control tuning
 * 
 * @author Dhyan Soni
 */
public class SysIdTuner {
    private final SysIdRoutine sysIdRoutine;
    private static Consumer<SysIdRoutineLog> setLog;
    private static Consumer<Voltage> setVolts;

    private VoltageDrive io;
    private Map<String, MotorData> motorData;

    // just use a list or map for this?
    public SysIdTuner(String name, SysIdRoutine.Config config, Subsystem subsystem, VoltageDrive io,
            Map<String, MotorData> motorData) {

        this.io = io;
        this.motorData = motorData;

        setVolts = (Voltage volts) -> setVoltage(volts);
        setLog = (SysIdRoutineLog) -> setLog(SysIdRoutineLog);

        sysIdRoutine = new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(setVolts, setLog, subsystem, name));
    }

    private void setLog(SysIdRoutineLog log) {
        motorData.forEach((motorName, data) -> {
            log.motor(motorName)
                    .voltage(Voltage.ofBaseUnits(data.appliedVolts, Volts))
                    .linearPosition(Meters.ofBaseUnits(data.position))
                    .linearVelocity(MetersPerSecond.ofBaseUnits(data.velocity))
                    .linearAcceleration(MetersPerSecondPerSecond.ofBaseUnits(data.acceleration));
        });

        // log.motor("motor").voltage(Voltage.ofBaseUnits(appliedVolts, Volts));
        // log.motor("motor").linearPosition(Meters.ofBaseUnits(position));
        // log.motor("motor").linearVelocity(MetersPerSecond.ofBaseUnits(velocity));
        // log.motor("motor").linearAcceleration(MetersPerSecondPerSecond.ofBaseUnits(acceleration));
    }

    public void setVoltage(Voltage volts) {
        try {
            io.setVoltage(volts.magnitude());
        } catch (Exception e) {
            System.err.println("Failed to set voltage: " + e.getMessage());
        }
    }

    public interface VoltageDrive {
        void setVoltage(double volts);
    }

    public static class MotorData {
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}


/*
 *  // Motor data obj
        Map<String, MotorData> motorData = Map.of(
            "motor", new MotorData(
                0.0, // initial appliedVolts
                0.0, // initial position
                0.0, // initial velocity
                0.0  // initial acceleration
            )
        );

    // Configuration, here are previously tested values
    SysIdRoutine.Config config = new SysIdRoutine.Config(
        Volts.per(Seconds).of(1.2), // Voltage ramp rate
        Volts.of(12),              // Max voltage
        Seconds.of(10)             // Test duration
    );

    // Instantiate SysIdTuner
    sysIdTuner = new SysIdTuner(config, this, motorController, motorData);
 */
