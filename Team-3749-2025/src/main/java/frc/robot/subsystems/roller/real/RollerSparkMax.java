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

    public RollerSparkMax(Implementations implementation) {

        switch (implementation) {
            case ALGAE:
                rollerMotor = new OptixSpark(Algae.motorId, OptixSpark.Type.SPARKMAX);
                rollerMotor.setPositionConversionFactor(2 * Math.PI / Algae.gearRatio);
                rollerMotor.setVelocityConversionFactor((2 * Math.PI / Algae.gearRatio) / 60.0);
                rollerMotor.setInverted(Algae.inverted);
                rollerMotor.setPID(Algae.kPPosition, Algae.kIPosition, Algae.kDPosition, ClosedLoopSlot.kSlot0);
                rollerMotor.setPID(Algae.kPVelocity, Algae.kIVelocity, Algae.kDVelocity, ClosedLoopSlot.kSlot1);
                break;
            case SCORING:
                rollerMotor = new OptixSpark(Scoring.motorId, OptixSpark.Type.SPARKMAX);
                rollerMotor.setPositionConversionFactor(2 * Math.PI / Scoring.gearRatio);
                rollerMotor.setVelocityConversionFactor((2 * Math.PI / Scoring.gearRatio) / 60.0);
                rollerMotor.setInverted(Scoring.inverted);
                rollerMotor.setPID(Scoring.kPPosition, Scoring.kIPosition, Scoring.kDPosition, ClosedLoopSlot.kSlot0);
                rollerMotor.setPID(Scoring.kPVelocity, Scoring.kIVelocity, Scoring.kDVelocity, ClosedLoopSlot.kSlot1);

                break;
            case CORAL:
                rollerMotor = new OptixSpark(Coral.motorId, OptixSpark.Type.SPARKMAX);
                rollerMotor.setPositionConversionFactor(2 * Math.PI / Coral.gearRatio);
                rollerMotor.setVelocityConversionFactor((2 * Math.PI / Coral.gearRatio) / 60.0);
                rollerMotor.setInverted(Coral.inverted);
                rollerMotor.setPID(Coral.kPPosition, Coral.kIPosition, Coral.kDPosition, ClosedLoopSlot.kSlot0);
                rollerMotor.setPID(Coral.kPVelocity, Coral.kIVelocity, Coral.kDVelocity, ClosedLoopSlot.kSlot1);
                break;

            default:
                rollerMotor = new OptixSpark(0, OptixSpark.Type.SPARKMAX);

        }
        rollerMotor.setCurrentLimit(MotorControllerConstants.relaxedStallLimit,
                MotorControllerConstants.relaxedFreeLimit);
        rollerMotor.setBrakeMode(true);

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
