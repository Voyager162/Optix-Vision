package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorSimulation implements ElevatorIO {
    double measurementStdDevs;
    double carriageMassKg;
    double drumRadiusMeters;
    double startingHeightMeters = Units.feetToMeters(3.5);

    private final ElevatorSim driveSimSystem = new ElevatorSim(
        DCMotor.getNEO(2),
        6.0,
        carriageMassKg,
        drumRadiusMeters,
        Units.feetToMeters(3.5),
        Units.feetToMeters(6),
        true,
        Units.feetToMeters(3.5)
    );
    
    ElevatorSimulation (){}
}
