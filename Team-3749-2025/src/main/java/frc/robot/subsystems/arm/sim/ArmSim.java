package frc.robot.subsystems.arm.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * IO implementation for an arm subsystem's simulation
 * 
 * @author Weston Gardner
 */

public class ArmSim implements ArmIO {

    SingleJointedArmSim arm = new SingleJointedArmSim(

        DCMotor.getNEO(1), 
        6, 
        0.04, 
        ArmConstants.armLength_meters, 
        ArmConstants.armSimMinAngle_degrees, 
        ArmConstants.armSimMaxAngle_degrees, 
        true, 
        ArmConstants.armSimStartingAngle_degrees, 
        0.0

    );

    private double appliedVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;
    private double conversionFactor = 1;

    public ArmSim() {
        System.out.println("[Init] Creating ArmSim");
    }

    /**
     * Updates the set of loggable inputs for the sim.
     * @param data
     */

    @Override
    public void updateData(ArmData data) {
        arm.update(SimConstants.loopPeriodSec);

        previousVelocity = velocity;
        velocity = arm.getVelocityRadPerSec() * conversionFactor;
        data.positionUnits += velocity * 0.02;
        data.velocityUnits = velocity;
        data.accelerationUnits = (velocity - previousVelocity) / 0.02;
        data.currentAmps = arm.getCurrentDrawAmps();
        data.inputVolts = appliedVolts;
        data.appliedVolts = appliedVolts;

        data.tempCelcius = 0.0; // sim has no temperature
    }

    /**
     * Run the motor at the specified voltage.
     * @param volts
     */

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        arm.setInputVoltage(appliedVolts);
    }
}
