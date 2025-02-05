package frc.robot.subsystems.roller.real;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerConstants.Algae;
import frc.robot.subsystems.roller.RollerConstants.Coral;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.Scoring;
import frc.robot.utils.OptixSpark;
import frc.robot.utils.MiscConstants.MotorControllerConstants;

public class RollerSparkMax implements RollerIO {
    private OptixSpark rollerMotor;

    public RollerSparkMax(Implementations implementation, int motorID, double positionConversionFactor,
            boolean isInverted) {
        rollerMotor = new OptixSpark(motorID, OptixSpark.Type.SPARKMAX);
        rollerMotor.setCurrentLimit(MotorControllerConstants.relaxedStallLimit,
                MotorControllerConstants.relaxedFreeLimit);
        rollerMotor.setInverted(isInverted);
        rollerMotor.setBrakeMode(true);
        rollerMotor.setPositionConversionFactor(positionConversionFactor);
        rollerMotor.setVelocityConversionFactor(positionConversionFactor / 60.0);

        switch (implementation) {
            case ALGAE:
                rollerMotor.setPID(Algae.kPPosition, Algae.kIPosition, Algae.kDPosition, ClosedLoopSlot.kSlot0);
                rollerMotor.setPID(Algae.kPVelocity, Algae.kIVelocity, Algae.kDVelocity, ClosedLoopSlot.kSlot1);

                break;
            case SCORING:
                rollerMotor.setPID(Scoring.kPPosition, Scoring.kIPosition, Scoring.kDPosition, ClosedLoopSlot.kSlot0);
                rollerMotor.setPID(Scoring.kPVelocity, Scoring.kIVelocity, Scoring.kDVelocity, ClosedLoopSlot.kSlot1);

                break;
            case CORAL:
                rollerMotor.setPID(Coral.kPPosition, Coral.kIPosition, Coral.kDPosition, ClosedLoopSlot.kSlot0);
                rollerMotor.setPID(Coral.kPVelocity, Coral.kIVelocity, Coral.kDVelocity, ClosedLoopSlot.kSlot1);

                break;

            default:
                break;
        }

    }

    @Override
    public void setBrakeMode(boolean enabled) {
        rollerMotor.setBrakeMode(enabled);
    }

    @Override
    public void updateData(RollerData data) {
        data.rollerAppliedVolts = rollerMotor.getAppliedVolts();
        data.rollerVelocityRadPerSec = rollerMotor.getVelocity();
        data.rollerTempCelcius = rollerMotor.getTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        rollerMotor.setVoltage(volts);
    }

    @Override
    public void setVelocity(double setpointVelocity, double feedforward) {
        rollerMotor.setVelocityControl(setpointVelocity, feedforward);
    }

    @Override
    public void setPosition(double setpointVelocity, double feedforward) {
        rollerMotor.setVelocityControl(setpointVelocity, feedforward);
    }
}
