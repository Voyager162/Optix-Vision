package frc.robot.subsystems.elevator.real;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.Consumer;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.utils.MiscConstants.SimConstants;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;

import static edu.wpi.first.units.MutableMeasure.*;
import static edu.wpi.first.units.Units.*;

/**
 * SparkMax for elevator subsystem
 * 
 * @author Dhyan Soni
 */

public class ElevatorSparkMax implements ElevatorIO {
    private SparkMax leftMotor = new SparkMax(ElevatorConstants.ElevatorSpecs.motorIds[0], MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(ElevatorConstants.ElevatorSpecs.motorIds[1], MotorType.kBrushless);
    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    private SparkAbsoluteEncoder absoluteEncoder;
    private double absolutePos;

    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    public ElevatorSparkMax() {
        System.out.println("[Init] Creating Elevator");

        leftConfig.smartCurrentLimit(ElevatorConstants.ElevatorSpecs.stallLimit,
                ElevatorConstants.ElevatorSpecs.freeLimit);
        leftConfig.encoder.inverted(false);
        leftConfig.inverted(false);
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.encoder.positionConversionFactor(2 * Math.PI);
        leftConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightConfig.apply(leftConfig);
        rightConfig.encoder.inverted(true);
        rightConfig.inverted(true);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = leftMotor.getAbsoluteEncoder();
        absolutePos = absoluteEncoder.getPosition() + ElevatorConstants.ElevatorSpecs.zeroOffset;

        leftMotor.getEncoder().setPosition(absolutePos);
        rightMotor.getEncoder().setPosition(absolutePos);

        double positionMeters = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getVelocity()) / 2;
        double velocityMetersPerSecond = velocity;
        double accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        double leftCurrentAmps = leftMotor.getOutputCurrent();
        double rightCurrentAmps = rightMotor.getOutputCurrent();
        double leftAppliedVolts = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
        double rightAppliedVolts = rightMotor.getBusVoltage() * rightMotor.getAppliedOutput();
        double leftTempCelcius = leftMotor.getMotorTemperature();
        double rightTempCelcius = rightMotor.getMotorTemperature();
        // Voltage volts = Voltage.ofRelativeUnits(12.0, VoltageUnit.class);
        // ; // applied volts
        // Distance distance; // position meters
        // LinearVelocity velocity2; // velocity

        // double velocity = (leftMotor.getEncoder().getVelocity() + rightMotor.getEncoder().getVelocity()) / 2;
        // Consumer<SysIdRoutineLog> m_log = (SysIdRoutineLog log) -> {
        //     new SysIdRoutine.Mechanism(
        //         this::identificationDriveConsumer,
        //             // Record a frame for the left motors. Since these share an encoder, we consider
        //             // the entire group to be one motor.
        //         log -> {
        //             SmartDashboard.putNumber(
        //             "motorAppliedVolts",
        //             identificationVoltageMeasure
        //                 .mut_replace(modules[0].getModuleData().driveAppliedVolts, Volts)
        //                 .magnitude()
        //             );
        //             SmartDashboard.putNumber(
        //             "motorSpeed",
        //             identificaitonVelocityMeasure
        //                 .mut_replace(
        //                 modules[0].getModuleData().driveVelocityMPerSec,
        //                 MetersPerSecond
        //                 )
        //                 .magnitude()
        //             );

        //             log
        //             .motor("front-left")
        //             .voltage(
        //                 identificationVoltageMeasure.mut_replace(
        //                 modules[0].getModuleData().driveAppliedVolts,
        //                 Volts
        //                 )
        //             )
        //             .linearPosition(
        //                 identificationDistanceMeasure.mut_replace(
        //                 modules[0].getModuleData().drivePositionM,
        //                 Meters
        //                 )
        //             )
        //             .linearVelocity(
        //                 identificaitonVelocityMeasure.mut_replace(
        //                 modules[0].getModuleData().driveVelocityMPerSec,
        //                 MetersPerSecond
        //                 )
        //             );
        //     log.motor("elevator-left")
        //         .voltage(volts)
        //         .linearPosition(distance)
        //         .linearVelocity(velocity2);
        //     // Record a frame for the right motors.  Since these share an encoder, we consider
        //     // the entire group to be one motor.
        //     log.motor("elevator-right")
        //         .voltage(
        //             m_appliedVoltage.mut_replace(
        //                 m_rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
        //         .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))
        //         .linearVelocity(
        //             m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
        };
    
    
        
    public void setIdleMode(IdleMode idleMode) {
        leftConfig.idleMode(idleMode);
        rightConfig.idleMode(idleMode);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateData(ElevatorData data) {
        previousVelocity = velocity;
        velocity = (leftMotor.getEncoder().getVelocity() + rightMotor.getEncoder().getVelocity()) / 2;
        // data.positionMeters = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getVelocity()) / 2
        //         + absolutePos;
        data.positionMeters = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getVelocity()) / 2;
        data.velocityMetersPerSecond = velocity;
        data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.leftCurrentAmps = leftMotor.getOutputCurrent();
        data.rightCurrentAmps = rightMotor.getOutputCurrent();
        data.inputVolts = inputVolts;
        data.leftAppliedVolts = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
        data.rightAppliedVolts = rightMotor.getBusVoltage() * rightMotor.getAppliedOutput();
        data.leftTempCelcius = leftMotor.getMotorTemperature();
        data.rightTempCelcius = rightMotor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12, 12);
        leftMotor.setVoltage(inputVolts);
        rightMotor.setVoltage(inputVolts);
    }
}
 