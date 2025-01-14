package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * SparkMax for elevator subsystem
 * A
 * @author Dhyan Soni
 */

public class ElevatorSparkMax implements ElevatorIO {
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.ElevatorSpecs.motorIds[0], MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ElevatorConstants.ElevatorSpecs.motorIds[1], MotorType.kBrushless);
    private SparkMaxConfig leftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightConfig = new SparkMaxConfig();

    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    ElevatorSparkMax (){
        System.out.println("[Init] Creating Elevator");

        leftConfig.smartCurrentLimit(30, 50);
        leftConfig.encoder.inverted(false);
        leftConfig.inverted(false);
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.encoder.positionConversionFactor(2 * Math.PI);
        leftConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftConfig.smartCurrentLimit(30, 50);
        rightConfig.encoder.inverted(true);
        rightConfig.inverted(true);
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.encoder.positionConversionFactor(2 * Math.PI);
        leftConfig.encoder.velocityConversionFactor(2 * Math.PI / 60.0);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIdleMode(IdleMode idleMode){
        leftConfig.idleMode(idleMode);
        rightConfig.idleMode(idleMode);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateData(ElevatorData data) {
        previousVelocity = velocity;
        velocity = (leftMotor.getEncoder().getVelocity() + rightMotor.getEncoder().getVelocity()) / 2;
        data.positionMeters = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getVelocity()) / 2;
        data.velocityUnits = velocity;
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
    public void setVoltage(double volts){
        inputVolts = MathUtil.clamp(volts, -12, 12);
        // inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        leftMotor.setVoltage(inputVolts);
        rightMotor.setVoltage(inputVolts);
    } 
}
