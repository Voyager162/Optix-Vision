package frc.robot.subsystems.elevator.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.utils.MiscConstants.SimConstants;

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
        ElevatorConstants.ElevatorSpecs.startingHeightMeters
    );

    public ElevatorSimulation (){
        System.out.println("[Init] Creating ElevatorSimulation");
    }

    @Override
    public void updateData(ElevatorData data) {
        elevatorSimSystem.update(0.02);
        previousVelocity = velocity;
        velocity = elevatorSimSystem.getVelocityMetersPerSecond();
        data.positionMeters = elevatorSimSystem.getPositionMeters();
        data.velocityUnits = elevatorSimSystem.getVelocityMetersPerSecond();
        data.accelerationUnits = (velocity - previousVelocity) / SimConstants.loopPeriodSec;;
        data.inputVolts = inputVolts;
        data.appliedVolts = inputVolts;
        data.currentAmps = elevatorSimSystem.getCurrentDrawAmps();
        data.tempCelcius = 0; // Sim has no temp
    }

    @Override
    public void setVoltage(double volts){
        inputVolts = MathUtil.clamp(volts, -12, 12);
        // inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        elevatorSimSystem.setInputVoltage(inputVolts);
    } 
}
