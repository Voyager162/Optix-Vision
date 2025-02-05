package frc.robot.subsystems.roller.real;

import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.roller.RollerIO;
import frc.robot.utils.OptixSpark;
import frc.robot.utils.MiscConstants.MotorControllerConstants;

public class RollerSparkMax implements RollerIO {
    private OptixSpark rollerMotor;

    public RollerSparkMax(int motorID, double positionConversionFactor, boolean isInverted) {
        rollerMotor = new OptixSpark(motorID, OptixSpark.Type.SPARKMAX);
        rollerMotor.setCurrentLimit(MotorControllerConstants.relaxedStallLimit,
                MotorControllerConstants.relaxedFreeLimit);
        rollerMotor.setInverted(isInverted);
        rollerMotor.setBrakeMode(true);
        rollerMotor.setPositionConversionFactor(positionConversionFactor);
        rollerMotor.setVelocityConversionFactor(positionConversionFactor / 60.0);

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
}
