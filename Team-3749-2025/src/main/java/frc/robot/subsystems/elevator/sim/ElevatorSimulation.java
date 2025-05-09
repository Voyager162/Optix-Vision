package frc.robot.subsystems.elevator.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.utils.MiscConstants.SimConstants;
import frc.robot.utils.UtilityFunctions;

/**
 * Simulation for elevator subsystem
 * 
 * @author Dhyan Soni
 */

public class ElevatorSimulation implements ElevatorIO {
    private double inputVolts = 0;
    private double previousVelocity = 0;
    private double velocity = 0;

    private final ElevatorSim elevatorSimSystem = new ElevatorSim(
            DCMotor.getNEO(2),
            ElevatorConstants.ElevatorSpecs.gearing,
            ElevatorConstants.ElevatorSpecs.carriageMassKg,
            ElevatorConstants.ElevatorSpecs.drumRadiusMeters,
            ElevatorConstants.ElevatorSpecs.minHeightMeters,
            ElevatorConstants.ElevatorSpecs.maxHeightMeters,
            true,
            ElevatorConstants.ElevatorSpecs.startingHeightMeters);

    public ElevatorSimulation() {
        System.out.println("[Init] Creating ElevatorSimulation");
    }

    @Override
    public void updateData(ElevatorData data) {
        elevatorSimSystem.update(0.02);
        previousVelocity = velocity;
        velocity = elevatorSimSystem.getVelocityMetersPerSecond();
        data.positionMeters = elevatorSimSystem.getPositionMeters();
        data.velocityMetersPerSecond = velocity;
        data.accelerationMetersPerSecondSquared = (velocity - previousVelocity) / SimConstants.loopPeriodSec;
        data.leftAppliedVolts = inputVolts;
        data.rightAppliedVolts = inputVolts;
        data.leftCurrentAmps = elevatorSimSystem.getCurrentDrawAmps();
        data.rightCurrentAmps = data.leftCurrentAmps;
        data.leftTempCelcius = 0;
        data.rightTempCelcius = data.leftTempCelcius;
    }

    @Override
    public void setVoltage(double volts) {
        inputVolts = UtilityFunctions.applyDeadband(inputVolts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12, 12);
        elevatorSimSystem.setInputVoltage(inputVolts);
        SmartDashboard.putNumber("input volts test", volts);
    }
}
