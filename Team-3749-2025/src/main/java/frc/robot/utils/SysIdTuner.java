package frc.robot.utils;

import java.util.function.Consumer;

import org.ejml.dense.block.VectorOps_DDRB;

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

    private Subsystem subsystem;

    private double appliedVolts;
    private double position;
    private double velocity;
    private double acceleration;

    // just use a list or map for this?
    public SysIdTuner(String name, Subsystem subsystem, double appliedVolts, double position, double velocity,
            double acceleration) {
        this.subsystem = subsystem;

        this.appliedVolts = appliedVolts;
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;

        setVolts = (Voltage volts) -> setVoltage(volts);
        setLog = (SysIdRoutineLog) -> setLog(SysIdRoutineLog);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.per(Seconds).of(1.2), Volts.of(12), Seconds.of(10)),
                new SysIdRoutine.Mechanism(setVolts, setLog, subsystem, name));
    }

    private void setLog(SysIdRoutineLog log) {
        log.motor("motor").voltage(Voltage.ofBaseUnits(appliedVolts, Volts));
        log.motor("motor").linearPosition(Meters.ofBaseUnits(position));
        log.motor("motor").linearVelocity(MetersPerSecond.ofBaseUnits(velocity));
        log.motor("motor").linearAcceleration(MetersPerSecondPerSecond.ofBaseUnits(acceleration));
    }

    // ask user to make this? how can i import the IO
    public void setVoltage(Voltage volts) {
        io.setVoltage(volts.magnitude());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
