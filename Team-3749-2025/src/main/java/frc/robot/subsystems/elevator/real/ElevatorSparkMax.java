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
    private OptixSpark leftMotorLead = new OptixSpark(ElevatorConstants.ElevatorSpecs.motorIds[0],
            OptixSpark.Type.SPARKMAX);

    private OptixSpark rightMotorFollow = new OptixSpark(ElevatorConstants.ElevatorSpecs.motorIds[1],
            OptixSpark.Type.SPARKMAX);

    private double previousVelocity = 0;

    public ElevatorSparkMax() {
        System.out.println("[Init] Creating Elevator");

        leftMotorLead.setPositionConversionFactor(Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / ElevatorConstants.ElevatorSpecs.gearing);
        leftMotorLead.setVelocityConversionFactor(Math.PI * 2 * ElevatorConstants.ElevatorSpecs.drumRadiusMeters
                / (60 * ElevatorConstants.ElevatorSpecs.gearing));

        leftMotorLead.setCurrentLimit(MotorControllerConstants.standardStallLimit,
                MotorControllerConstants.standardFreeLimit);
        leftMotorLead.setInverted(ElevatorConstants.ElevatorSpecs.motorInverted[0]);
        leftMotorLead.setBrakeMode(false);

        leftMotorLead.setPID(ElevatorControl.kP.get(), ElevatorControl.kI.get(), ElevatorControl.kD.get(), ClosedLoopSlot.kSlot0);

        rightMotorFollow.applyConfig(leftMotorLead.getConfig());
        rightMotorFollow.setInverted(ElevatorConstants.ElevatorSpecs.motorInverted[1]);
        rightMotorFollow.setFollow(leftMotorLead);

    }

    @Override
    public void setBrakeMode(boolean enabled) {
        rightMotorFollow.setBrakeMode(enabled);
        leftMotorLead.setBrakeMode(enabled);
    }

    @Override
    public void updateData(ElevatorData data) {
        double velocity = (leftMotorLead.getVelocity() + rightMotorFollow.getVelocity()) / 2;
        // data.positionMeters = (leftMotor.getEncoder().getPosition() +
        // rightMotor.getEncoder().getVelocity()) / 2
        // + absolutePos;
        data.positionMeters = (leftMotorLead.getPosition() + rightMotorFollow.getPosition()) / 2;
        data.velocityMetersPerSecond = velocity;
        data.accelerationMetersPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.leftCurrentAmps = leftMotorLead.getCurrent();
        data.rightCurrentAmps = rightMotorFollow.getCurrent();
        data.leftAppliedVolts = leftMotorLead.getAppliedVolts();
        data.rightAppliedVolts = rightMotorFollow.getAppliedVolts();
        data.leftTempCelcius = leftMotorLead.getTemperature();
        data.rightTempCelcius = rightMotorFollow.getTemperature();

        previousVelocity = velocity;
    }

    @Override
    public void setVoltage(double volts) {
        double inputVolts = MathUtil.applyDeadband(volts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12, 12);
        leftMotorLead.setVoltage(inputVolts);
        rightMotorFollow.setVoltage(inputVolts);
    }

    @Override
    public void setPosition(double setpointPosition, double feedforward) {
        leftMotorLead.setPositionControl(setpointPosition, feedforward);
    }

}
