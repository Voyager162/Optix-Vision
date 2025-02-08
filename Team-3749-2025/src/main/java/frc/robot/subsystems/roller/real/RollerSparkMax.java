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
import frc.robot.utils.LoggedTunableNumber;

public class RollerSparkMax implements RollerIO {
    private OptixSpark rollerMotor;

    protected LoggedTunableNumber kp;
    protected LoggedTunableNumber ki;
    protected LoggedTunableNumber kd;

    public RollerSparkMax(Implementations implementation) {

        switch (implementation) {
            case ALGAE:
                kp = new LoggedTunableNumber("AlgaeRoller" + "/kP", Algae.kPVelocity);
                ki = new LoggedTunableNumber("AlgaeRoller" + "/kI", Algae.kIVelocity);
                kd = new LoggedTunableNumber("AlgaeRoller" + "/kD", Algae.kDVelocity);
                Algae.kPVelocity = kp.get();
                Algae.kIVelocity = ki.get();
                Algae.kDVelocity = kd.get();

                kp = new LoggedTunableNumber("AlgaeRoller" + "/kP", Algae.kPPosition);
                ki = new LoggedTunableNumber("AlgaeRoller" + "/kI", Algae.kIPosition);
                kd = new LoggedTunableNumber("AlgaeRoller" + "/kD", Algae.kDPosition);
                Algae.kPPosition = kp.get();
                Algae.kIPosition = ki.get();
                Algae.kDPosition = kd.get();

                rollerMotor = new OptixSpark(Algae.motorId, OptixSpark.Type.SPARKMAX);
                rollerMotor.setPositionConversionFactor(2 * Math.PI / Algae.gearRatio);
                rollerMotor.setVelocityConversionFactor((2 * Math.PI / Algae.gearRatio) / 60.0);
                rollerMotor.setInverted(Algae.inverted);
                rollerMotor.setPID(Algae.kPPosition, Algae.kIPosition, Algae.kDPosition, ClosedLoopSlot.kSlot0);
                rollerMotor.setPID(Algae.kPVelocity, Algae.kIVelocity, Algae.kDVelocity, ClosedLoopSlot.kSlot1);
                break;
            case SCORING:
                kp = new LoggedTunableNumber("ScoringRoller" + "/kP", Scoring.kPVelocity);
                ki = new LoggedTunableNumber("ScoringRoller" + "/kI", Scoring.kIVelocity);
                kd = new LoggedTunableNumber("ScoringRoller" + "/kD", Scoring.kDVelocity);
                Scoring.kPVelocity = kp.get();
                Scoring.kIVelocity = ki.get();
                Scoring.kDVelocity = kd.get();

                kp = new LoggedTunableNumber("ScoringRoller" + "/kP", Scoring.kPPosition);
                ki = new LoggedTunableNumber("ScoringRoller" + "/kI", Scoring.kIPosition);
                kd = new LoggedTunableNumber("ScoringRoller" + "/kD", Scoring.kDPosition);
                Scoring.kPPosition = kp.get();
                Scoring.kIPosition = ki.get();
                Scoring.kDPosition = kd.get();

                rollerMotor = new OptixSpark(Scoring.motorId, OptixSpark.Type.SPARKMAX);
                rollerMotor.setPositionConversionFactor(2 * Math.PI / Scoring.gearRatio);
                rollerMotor.setVelocityConversionFactor((2 * Math.PI / Scoring.gearRatio) / 60.0);
                rollerMotor.setInverted(Scoring.inverted);
                rollerMotor.setPID(Scoring.kPPosition, Scoring.kIPosition, Scoring.kDPosition, ClosedLoopSlot.kSlot0);
                rollerMotor.setPID(Scoring.kPVelocity, Scoring.kIVelocity, Scoring.kDVelocity, ClosedLoopSlot.kSlot1);
                break;
            case CORAL:
                kp = new LoggedTunableNumber("CoralRoller" + "/kP", Coral.kPVelocity);
                ki = new LoggedTunableNumber("CoralRoller" + "/kI", Coral.kIVelocity);
                kd = new LoggedTunableNumber("CoralRoller" + "/kD", Coral.kDVelocity);
                Scoring.kPVelocity = kp.get();
                Scoring.kIVelocity = ki.get();
                Scoring.kDVelocity = kd.get();

                kp = new LoggedTunableNumber("CoralRoller" + "/kP", Coral.kPPosition);
                ki = new LoggedTunableNumber("CoralRoller" + "/kI", Coral.kIPosition);
                kd = new LoggedTunableNumber("CoralRoller" + "/kD", Coral.kDPosition);
                Scoring.kPPosition = kp.get();
                Scoring.kIPosition = ki.get();
                Scoring.kDPosition = kd.get();

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
    public void setPosition(double setpointPosition, double feedforward) {
        rollerMotor.setPositionControl(setpointPosition, feedforward);
    }
}
