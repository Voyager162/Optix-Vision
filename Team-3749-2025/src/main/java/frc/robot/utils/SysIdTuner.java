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
import static edu.wpi.first.units.Units.Volts;;

/**
 * SysId assisted control tuning
 * 
 * @author Dhyan Soni
 */
public class SysIdTuner {
    private SysIdRoutine sysIdRoutine;
    private Consumer<SysIdRoutineLog> setLog;
    private Consumer<Voltage> setVolts;

    private VoltageDrive io;
    private Map<String, MotorData> motorData;

    /**
     * Setup SysId
     * 
     * @param name - name of the mechanism
     * @param config - SysIdRoutine.Config object
     * @param subsystem - Subsystem you are characterizing
     * @param io - VoltageDriv
     */
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

    /**
     * Update logs with voltage, linearPosition, linearVelocity, and linearAcceleration
     * @param log - SysIdRoutineLog object
     */
    private void setLog(SysIdRoutineLog log) {
        motorData.forEach((motorName, data) -> {
            // System.out.println("volts" + data.appliedVolts);
            // System.out.println("pos" + data.position);
            // System.out.println("vel" + data.velocity);
            // System.out.println("acceleration" + data.acceleration);
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

    /**
     * Set voltage method in the Voltage unit
     * 
     * @param volts - voltage to be applied
     */
    public void setVoltage(Voltage volts) {
        try {
            io.setVoltage(volts.magnitude());
        } catch (Exception e) {
            System.err.println("Failed to set voltage: " + e.getMessage());
        }
    }

    /*
     * Used as the type for all subsystem IOs
     */
    public interface VoltageDrive {
        void setVoltage(double volts);
    }

    /**
     * Command for Quasistatic test
     * @return command and direction for Quasistatic test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Command for Dynamic test
     * @return command and direction for Dynamic test
     */
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
