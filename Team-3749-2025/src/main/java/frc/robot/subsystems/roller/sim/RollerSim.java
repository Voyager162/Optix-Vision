package frc.robot.subsystems.roller.sim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Algae;
import frc.robot.subsystems.roller.RollerConstants.Coral;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.Scoring;
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
    private PIDController positionController;
    private PIDController velocityController;

    private double velocity = 0;
    private double position = 0;

    /**
     * Constructs a simulated roller subsystem based on the implementation type
     *
     * @param implementation 
     */
    public RollerSim(Implementations implementation) {
        switch (implementation) {
            case ALGAE:
                momentOfInertia = RollerConstants.Algae.momentOfInertia;
                gearRatio = RollerConstants.Algae.gearRatio;
                measurementNoise = RollerConstants.Algae.measurementNoise;
                positionController = new PIDController(Algae.kPPosition.get(), Algae.kIPosition.get(), Algae.kDPosition.get());
                velocityController = new PIDController(Algae.kPVelocity.get(), Algae.kIVelocity.get(), Algae.kDVelocity.get());
                break;
            case CORAL:
                momentOfInertia = RollerConstants.Coral.momentOfInertia;
                gearRatio = RollerConstants.Coral.gearRatio;
                measurementNoise = RollerConstants.Coral.measurementNoise;
                positionController = new PIDController(Coral.kPPosition.get(), Coral.kIPosition.get(), Coral.kDPosition.get());
                velocityController = new PIDController(Coral.kPVelocity.get(), Coral.kIVelocity.get(), Coral.kDVelocity.get());
                break;
            case SCORING:
                momentOfInertia = RollerConstants.Scoring.momentOfInertia;
                gearRatio = RollerConstants.Scoring.gearRatio;
                measurementNoise = RollerConstants.Scoring.measurementNoise;
                positionController = new PIDController(Scoring.kPPosition.get(), Scoring.kIPosition.get(), Scoring.kDPosition.get());
                velocityController = new PIDController(Scoring.kPVelocity.get(), Scoring.kIVelocity.get(), Scoring.kDVelocity.get());
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

    @Override
    public void setVelocity(double setpointVelocity, double feedforward) {
        rollerMotor.setInputVoltage(feedforward + velocityController.calculate(velocity, setpointVelocity));
    }

    /**
     * Sets the desired position of the roller by applying voltage
     */
    @Override
    public void setPosition(double setpointPosition, double feedforward) {
        rollerMotor.setInputVoltage(feedforward + positionController.calculate(position, setpointPosition));
    }
}
