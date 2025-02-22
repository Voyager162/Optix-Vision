package frc.robot.subsystems.elevator.real;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.utils.OptixSpark;
import frc.robot.utils.UtilityFunctions;
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
    @SuppressWarnings("unused")
    private double inputVolts = 0;

    public ElevatorSparkMax() {
        // System.out.println("[Init] Creating Elevator");
        // 2x for two stages, 2pi for circumfrence, radius of sprocket, gear ratio
        leftMotor.setPositionConversionFactor(2 * Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / ElevatorConstants.ElevatorSpecs.gearing);
        leftMotor.setVelocityConversionFactor(2 * Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / (60 * ElevatorConstants.ElevatorSpecs.gearing));

        leftMotor.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        leftMotor.setInverted(ElevatorConstants.ElevatorSpecs.motorInverted[0]);
        leftMotor.setBrakeMode(true);

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
        inputVolts = MathUtil.clamp(volts, -12, 12);
        inputVolts = UtilityFunctions.applyDeadband(inputVolts, MotorControllerConstants.deadbandVoltage);
        Logger.recordOutput("subsystems/elevator/input volts", inputVolts);

        leftMotor.setVoltage(inputVolts);
        rightMotor.setVoltage(inputVolts);
    }
}
