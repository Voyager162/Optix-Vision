package frc.robot.subsystems.climb.real;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.utils.OptixSpark;

public class ClimbSparkMax implements ClimbIO
{

    private OptixSpark climbMotor = new OptixSpark(ClimbConstants.ClimbSpecs.climbMotorPort,
            OptixSpark.Type.SPARKMAX);
    private boolean isClimbing = false;

    @Override
    public void setVoltage(double volts)
    {
        climbMotor.setVoltage(volts);
    }

    @Override
    public void updateData(ClimbData data) {
        data.isClimbing = this.isClimbing;
        data.tempCelcius = climbMotor.getTemperature();
        data.appliedVolts = climbMotor.getAppliedVolts();
        data.currentAmps = climbMotor.getCurrent();
        data.velocityRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(climbMotor.getVelocity());
        data.positionRadians = Units.rotationsToRadians(climbMotor.getPosition());
    };

    @Override
    public void setBrakeMode(boolean enable) {
        climbMotor.setBrakeMode(enable);
    }

    @Override
    public void stop(){
        climbMotor.setVoltage(0);
    }

    @Override
    public void setIsClimbing(boolean isClimbing)
    {
        this.isClimbing = isClimbing;
    }
}
