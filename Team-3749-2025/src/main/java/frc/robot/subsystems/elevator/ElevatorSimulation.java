package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorSimulation implements ElevatorIO {
    private double inputVolts = 0;

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

    ElevatorSimulation (){
        System.out.println("[Init] Creating ElevatorSimulation");
    }

    @Override
    public void updateData(ElevatorData data) {
        elevatorSimSystem.update(0.02);
        data.positionMeters = elevatorSimSystem.getPositionMeters();
        data.velocityUnits = elevatorSimSystem.getVelocityMetersPerSecond();
        data.accelerationUnits = 0;
        data.inputVolts = inputVolts;
        data.appliedVolts = 0;
        data.currentAmps = 0;
        // Sim has no temp
        data.tempCelcius = 0;

        
    }

    @Override
    public void setVoltage(double volts){
        inputVolts = MathUtil.clamp(volts, -12, 12);
        // inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        elevatorSimSystem.setInputVoltage(inputVolts);
        // main periodic
        // SmartDashboard.putNumber("applied volts: " + index, voltage);
    } 
}
