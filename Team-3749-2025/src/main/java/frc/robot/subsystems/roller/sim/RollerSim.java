package frc.robot.subsystems.roller.sim;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerIO;

/**
 * Simulation for the roller subsystem
 *
 * @author Lilian Wu
 */
public class RollerSim implements RollerIO {
    private FlywheelSim rollerMotor;
    private double momentOfInertia;
    private double gearRatio;
    private double measurementNoise;

    private double velocity = 0;
    private double position = 0;

    /**
     * Constructs a simulated roller subsystem based on the implementation type
     *
     * @param implementation 
     */
    public RollerSim(Implementations implementation) {
        switch (implementation) {
            // case ALGAE:
            //     momentOfInertia = RollerConstants.Algae.momentOfInertia;
            //     gearRatio = RollerConstants.Algae.gearRatio;
            //     measurementNoise = RollerConstants.Algae.measurementNoise;
            //     break;
            case CORAL:
                momentOfInertia = RollerConstants.Coral.momentOfInertia;
                gearRatio = RollerConstants.Coral.gearRatio;
                measurementNoise = RollerConstants.Coral.measurementNoise;
                break;
            case SCORING:
                momentOfInertia = RollerConstants.Scoring.momentOfInertia;
                gearRatio = RollerConstants.Scoring.gearRatio;
                measurementNoise = RollerConstants.Scoring.measurementNoise;
                break;
        }

        DCMotor motor = DCMotor.getNEO(1);
        // calculates how angular velocity changes over time with applied voltage
        LinearSystem<N1, N1, N1> flyWheelSystem = LinearSystemId.createFlywheelSystem(motor, momentOfInertia,
                gearRatio);
        rollerMotor = new FlywheelSim(flyWheelSystem, motor, measurementNoise);
    }

    @Override
    public void updateData(RollerData data) {
        rollerMotor.update(0.02);
        velocity = rollerMotor.getAngularVelocityRadPerSec();
        data.rollerAppliedVolts = rollerMotor.getInputVoltage();
        data.rollerVelocityRadPerSec = velocity;
        data.currentAmps = rollerMotor.getCurrentDrawAmps();
        data.rollerTempCelcius = 0.0;
        position += velocity * 0.02;
        data.rollerPositionRad = position;
    }

    @Override
    public void setVoltage(double rollerVolts) {
        rollerMotor.setInputVoltage(rollerVolts);
    }
}
