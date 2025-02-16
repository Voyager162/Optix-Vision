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
import frc.robot.utils.MiscConstants.SimConstants;
import frc.robot.utils.LoggedTunableNumber;

public class RollerSparkMax implements RollerIO {
    private OptixSpark rollerMotor;

    protected LoggedTunableNumber kp;
    protected LoggedTunableNumber ki;
    protected LoggedTunableNumber kd;

    public RollerSparkMax(Implementations implementation) {

        switch (implementation) {
            case ALGAE:

                rollerMotor = new OptixSpark(Algae.motorId, OptixSpark.Type.SPARKMAX);
                rollerMotor.setPositionConversionFactor(2 * Math.PI / Algae.gearRatio);
                rollerMotor.setVelocityConversionFactor((2 * Math.PI / Algae.gearRatio) / 60.0);
                rollerMotor.setInverted(Algae.inverted);
                break;
            case SCORING:

                rollerMotor = new OptixSpark(Scoring.motorId, OptixSpark.Type.SPARKMAX);
                rollerMotor.setPositionConversionFactor(2 * Math.PI / Scoring.gearRatio);
                rollerMotor.setVelocityConversionFactor((2 * Math.PI / Scoring.gearRatio) / 60.0);
                rollerMotor.setInverted(Scoring.inverted);
                break;
            case CORAL:
                rollerMotor = new OptixSpark(Coral.motorId, OptixSpark.Type.SPARKMAX);
                rollerMotor.setPositionConversionFactor(2 * Math.PI / Coral.gearRatio);
                rollerMotor.setVelocityConversionFactor((2 * Math.PI / Coral.gearRatio) / 60.0);
                rollerMotor.setInverted(Coral.inverted);
                break;

            default:
                rollerMotor = new OptixSpark(0, OptixSpark.Type.SPARKMAX);

        }
        rollerMotor.setCurrentLimit(MotorControllerConstants.relaxedStallLimit,
                MotorControllerConstants.relaxedFreeLimit);
        rollerMotor.setBrakeMode(false);

        rollerMotor.applyConfig();

        rollerMotor.applyConfig();

    }

    @Override
    public void setBrakeMode(boolean enabled) {
        rollerMotor.setBrakeMode(enabled);
    }

    @Override
    public void updateData(RollerData data) {
        data.rollerAppliedVolts = rollerMotor.getAppliedVolts();
        double previousVelocity = data.rollerVelocityRadPerSec;
        data.rollerVelocityRadPerSec = rollerMotor.getVelocity();
        data.rollerTempCelcius = rollerMotor.getTemperature();
        data.currentAmps = rollerMotor.getCurrent();
        data.rollerPositionRad = rollerMotor.getPosition();
        data.acceleration = (data.rollerVelocityRadPerSec - previousVelocity) / SimConstants.loopPeriodSec;
    }

    @Override
    public void setVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        rollerMotor.setVoltage(volts);
    }

    @Override
    public void setVelocity(double setpointVelocity, double feedforward) {
        rollerMotor.setVelocityControl(setpointVelocity, feedforward, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void setPosition(double setpointPosition, double feedforward) {
        rollerMotor.setPositionControl(setpointPosition, feedforward, ClosedLoopSlot.kSlot0);
    }
}
