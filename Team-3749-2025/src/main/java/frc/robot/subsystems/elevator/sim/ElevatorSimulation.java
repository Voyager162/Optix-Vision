package frc.robot.subsystems.elevator.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorControl;
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
            ElevatorConstants.ElevatorSpecs.startingHeightMeters);

    public ElevatorSimulation() {
        System.out.println("[Init] Creating ElevatorSimulation");
        // Consumer<SysIdRoutineLog> log = (SysIdRoutineLog log) -> {
        // log.motor("elevator-left")
        // .voltage(
        // elevatorSimSystem.getVelocityMetersPerSecond().mut_replace(
        // m_leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
        // .linearPosition(m_distance.mut_replace(m_leftEncoder.getDistance(), Meters))
        // .linearVelocity(
        // m_velocity.mut_replace(m_leftEncoder.getRate(), MetersPerSecond));
        // // Record a frame for the right motors. Since these share an encoder, we
        // consider
        // // the entire group to be one motor.
        // log.motor("elevator-right")
        // .voltage(
        // m_appliedVoltage.mut_replace(
        // m_rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
        // .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))
        // .linearVelocity(
        // m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
        // };
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

        // Sim has no temp
        data.leftTempCelcius = 0;
        data.rightTempCelcius = data.leftTempCelcius;
    }

    @Override
    public void setVoltage(double volts) {
        inputVolts = MathUtil.applyDeadband(inputVolts, 0.05);
        inputVolts = MathUtil.clamp(volts, -12, 12);
        elevatorSimSystem.setInputVoltage(inputVolts);
    }



}
