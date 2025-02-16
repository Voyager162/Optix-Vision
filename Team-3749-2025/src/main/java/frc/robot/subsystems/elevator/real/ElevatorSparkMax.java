package frc.robot.subsystems.elevator.real;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorControl;
import frc.robot.utils.OptixSpark;
import frc.robot.utils.MiscConstants.MotorControllerConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * SparkMax for elevator subsystem
 * 
 * @author Dhyan Soni
 */

public class ElevatorSparkMax implements ElevatorIO {
    private OptixSpark leftMotor = new OptixSpark(ElevatorConstants.ElevatorSpecs.motorIds[0],
            OptixSpark.Type.SPARKMAX);

    private OptixSpark rightMotor = new OptixSpark(ElevatorConstants.ElevatorSpecs.motorIds[1],
            OptixSpark.Type.SPARKMAX);

    private double previousVelocity = 0;

    public ElevatorSparkMax() {
        // System.out.println("[Init] Creating Elevator");

        leftMotor.setPositionConversionFactor(Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / ElevatorConstants.ElevatorSpecs.gearing);
        leftMotor.setVelocityConversionFactor(Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / (60 * ElevatorConstants.ElevatorSpecs.gearing));

        leftMotor.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        leftMotor.setInverted(ElevatorConstants.ElevatorSpecs.motorInverted[0]);
        leftMotor.setBrakeMode(false);

        leftMotor.applyConfig();
        rightMotor.applyConfig(leftMotor.getConfig());
        rightMotor.setInverted(ElevatorConstants.ElevatorSpecs.motorInverted[1]);
        rightMotor.applyConfig();

    }

    @Override
    public void setBrakeMode(boolean enabled) {
        rightMotor.setBrakeMode(enabled);
        leftMotor.setBrakeMode(enabled);
    }

    @Override
    public void updateData(ElevatorData data) {
        double velocity = (leftMotor.getVelocity() + rightMotor.getVelocity()) / 2;
        // data.positionMeters = (leftMotor.getEncoder().getPosition() +
        // rightMotor.getEncoder().getVelocity()) / 2
        // + absolutePos;
        data.positionMeters = (leftMotor.getPosition() + rightMotor.getPosition()) / 2;
        data.velocityMetersPerSecond = velocity;
        data.accelerationMetersPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.leftCurrentAmps = leftMotor.getCurrent();
        data.rightCurrentAmps = rightMotor.getCurrent();
        data.leftAppliedVolts = leftMotor.getAppliedVolts();
        data.rightAppliedVolts = rightMotor.getAppliedVolts();
        data.leftTempCelcius = leftMotor.getTemperature();
        data.rightTempCelcius = rightMotor.getTemperature();

        previousVelocity = velocity;
    }

    @Override
    public void setVoltage(double volts) {
        double inputVolts = MathUtil.applyDeadband(volts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12, 12);
        leftMotor.setVoltage(inputVolts);
        rightMotor.setVoltage(inputVolts);
    }


}
